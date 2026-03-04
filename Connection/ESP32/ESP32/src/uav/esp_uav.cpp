#include <Arduino.h> // pulls in all the basic serial and time stuff
#include <WiFi.h> // needed for the radio hardware to even turn on
#include <esp_now.h> // the actual radio api for sending raw bytes

#include "freertos/FreeRTOS.h" // the foundation for the multitasking
#include "freertos/task.h" // lets us hire background workers
#include "freertos/queue.h" // the mailboxes for safer data handoffs

// mac address of the ugv (ground station) so we know where to scream
static uint8_t UGV_MAC[6] = {0xF8, 0xB3, 0xB7, 0x20, 0x69, 0xC0}; 

// binary markers so we can find the start of a real message in the usb noise
static const uint8_t SOF = 0xAA; // 170 (10101010) start of packet
static const uint8_t TYPE_TELEM = 1; // code for status data
static const uint8_t TYPE_CMD   = 2; // code for instructions
static const uint8_t TYPE_MSG   = 3; // code for raw text

// matches the memory layout of the python telemetry packet
typedef struct __attribute__((packed)) {
  uint32_t seq;    // counter for missing packets
  uint32_t t_ms;   // uav time clock
  float vx;        // speed data
  float vy;        // speed data
  uint8_t marker;  // true/false flag
  uint8_t estop;   // abort mission flag
} TelemetryPayload;

// matches the memory layout for wheel robot commands
typedef struct __attribute__((packed)) {
  uint32_t cmdSeq; // unique id for the move
  uint8_t  cmd;    // 1=arm, 2=disarm, etc
  uint8_t  estop;  // emergency stop bit
} CommandPayload;

// mailboxes for the background tasks to pass bytes around
static QueueHandle_t qTelemToNow  = nullptr; // usb -> radio
static QueueHandle_t qMsgToNow    = nullptr; // usb strings -> radio
static QueueHandle_t qCmdToNow    = nullptr; // commands for ground 
static QueueHandle_t qTelemToSerial = nullptr; // radio -> usb 
static QueueHandle_t qCmdToSerial = nullptr; // instructions from radio
static QueueHandle_t qMsgToSerial = nullptr; // text caught from air

// lock to stop workers from typing on the usb wire at the same time
static SemaphoreHandle_t serialMutex = nullptr; 

// math to check if the data got scrambled during the radio hop
static uint8_t checksum_xor(uint8_t type, uint8_t len, const uint8_t* payload) {
  uint8_t c = type ^ len; // mix the header
  for (uint8_t i = 0; i < len; i++) c ^= payload[i]; // xor every byte
  return c; // result is the digital seal
}

// wraps raw data in the protocol frame so the jetson can decode it
static void serial_send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t chk = checksum_xor(type, len, payload); // calculate seal
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) { // wait for access
    Serial.write(SOF);      // [1] start
    Serial.write(type);     // [2] what is this
    Serial.write(len);      // [3] how big
    if (len) Serial.write(payload, len); // [4] the meat
    Serial.write(chk);      // [5] checksum fingerprint
    xSemaphoreGive(serialMutex); // let others use the wire
  }
}

// triggers every time a packet hits the uav's antenna
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 1) return; // ignore ghost packets
  uint8_t fType = data[0]; // first byte is always the type

  // echo the radio event to the jetson for debugging
  char dbg[64]; // temporary buffer for string
  snprintf(dbg, 64, "ESP: Got Radio PKT type %d len %d", fType, len);
  if (qMsgToSerial) { // if mailbox exists
    uint8_t dBuf[64]; // data array
    memset(dBuf, 0, 64); // clear it
    memcpy(dBuf, dbg, strlen(dbg)); // copy text
    xQueueSend(qMsgToSerial, dBuf, 0); // ship it to usb task
  }

  // sort incoming radio data into the right mailbox
  if (fType == TYPE_TELEM && len >= (int)sizeof(TelemetryPayload)) {
    TelemetryPayload t; // create struct
    memcpy(&t, data + 1, sizeof(t)); // extract content
    if (qTelemToSerial) xQueueSend(qTelemToSerial, &t, 0); // hand off to usb
  }
  else if (fType == TYPE_CMD && len >= (int)sizeof(CommandPayload)) {
    CommandPayload cmd; // create struct
    memcpy(&cmd, data + 1, sizeof(cmd)); // extract content
    if (qCmdToSerial) xQueueSend(qCmdToSerial, &cmd, 0); // hand off to usb
  }
  else if (fType == TYPE_MSG) {
    uint8_t msgBuf[64]; // text buffer
    memset(msgBuf, 0, 64); // clean it
    uint8_t msgLen = (len - 1 > 64) ? 64 : len - 1; // clip if too long
    memcpy(msgBuf, data + 1, msgLen); // extract string
    if (qMsgToSerial) xQueueSend(qMsgToSerial, msgBuf, 0); // hand off to usb
  }
}

// logical states for unpacking the stream from the jetson usb
enum ParseState { WAIT_SOF, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHK };
typedef struct {
  ParseState st; // current mode
  uint8_t type; // expected packet type
  uint8_t len; // expected length
  uint8_t idx; // how many we got so far
  uint8_t buf[64]; // holding area
} SerialParser;

// resets the parser to hunt for the lead 0xAA byte
static void parser_init(SerialParser* p) { p->st = WAIT_SOF; }

// feeds one byte at a time through the grammar rules
static bool parser_step(SerialParser* p, uint8_t b, uint8_t* outType, uint8_t* outLen, uint8_t* outPayload) {
  switch (p->st) {
    case WAIT_SOF: if (b == SOF) p->st = WAIT_TYPE; break; // found start
    case WAIT_TYPE: p->type = b; p->st = WAIT_LEN; break; // got the label
    case WAIT_LEN:
      p->len = b; // save expected size
      if (p->len > 64) { parser_init(p); break; } // abort if it's too big
      p->idx = 0; // reset counter
      p->st = (p->len == 0) ? WAIT_CHK : WAIT_PAYLOAD; // move forward
      break;
    case WAIT_PAYLOAD:
      p->buf[p->idx++] = b; // collect data
      if (p->idx >= p->len) p->st = WAIT_CHK; // got it all
      break;
    case WAIT_CHK: {
      uint8_t expected = checksum_xor(p->type, p->len, p->buf); // check fingerprint
      if (b == expected) { // if valid 100%
        *outType = p->type; *outLen = p->len; // export info
        memcpy(outPayload, p->buf, p->len); // export payload
        parser_init(p); return true; // tell caller we got one
      }
      parser_init(p); break; // bad seal, restart
    }
  }
  return false; // still hunting
}

// Background Worker: Listens to the Jetson for instructions
void serialRxTask(void* pv) {
  SerialParser parser; // local gatekeeper
  parser_init(&parser); // reset state
  uint8_t fType, fLen, payload[64]; // temporary variables
  for (;;) { // infinite loop
    while (Serial.available() > 0) { // check if bytes are in usb buffer
      uint8_t b = (uint8_t)Serial.read(); // pull one byte
      if (parser_step(&parser, b, &fType, &fLen, payload)) { // logic check
        if (fType == TYPE_TELEM && fLen == sizeof(TelemetryPayload)) {
          TelemetryPayload t; memcpy(&t, payload, sizeof(t)); // extract
          if (qTelemToNow) xQueueSend(qTelemToNow, &t, 0); // hand to radio task
        }
        else if (fType == TYPE_CMD && fLen == sizeof(CommandPayload)) {
          CommandPayload c; memcpy(&c, payload, sizeof(c)); // extract
          if (qCmdToNow) xQueueSend(qCmdToNow, &c, 0); // hand to radio task
        }
        else if (fType == TYPE_MSG) {
            uint8_t msgBuf[64]; memset(msgBuf, 0, 64); // extract
            memcpy(msgBuf, payload, fLen); // hand to radio task
            if (qMsgToNow) xQueueSend(qMsgToNow, msgBuf, 0);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // wait 2ms before checking again
  }
}

// Background Worker: Blasts status updates over the air
void radioTxTelemTask(void* pv) {
  TelemetryPayload t; // create storage
  uint8_t pkt[1 + sizeof(TelemetryPayload)]; // packet buffer
  pkt[0] = TYPE_TELEM; // header id
  for (;;) { // infinite loop
    if (xQueueReceive(qTelemToNow, &t, portMAX_DELAY) == pdTRUE) { // wait for mailbox
      memcpy(pkt + 1, &t, sizeof(t)); // load payload
      esp_now_send(UGV_MAC, pkt, sizeof(pkt)); // blast it out
    }
  }
}

// Background Worker: Blasts mission commands over the air
void radioTxCmdTask(void* pv) {
  CommandPayload c; // create storage
  uint8_t pkt[1 + sizeof(CommandPayload)]; // packet buffer
  pkt[0] = TYPE_CMD; // header id
  for (;;) { // infinite loop
    if (xQueueReceive(qCmdToNow, &c, portMAX_DELAY) == pdTRUE) { // wait for mailbox
      memcpy(pkt + 1, &c, sizeof(c)); // load payload
      esp_now_send(UGV_MAC, pkt, sizeof(pkt)); // blast it out
    }
  }
}

// Background Worker: Blasts talk/debug text over the air
void radioTxMsgTask(void* pv) {
  uint8_t payload[64]; // raw byte buffer
  uint8_t pkt[1 + 64]; // total size
  pkt[0] = TYPE_MSG; // header id
  for (;;) { // infinite loop
    if (xQueueReceive(qMsgToNow, payload, portMAX_DELAY) == pdTRUE) { // wait for text
      uint8_t len = 0; while (len < 64 && payload[len] != 0) len++; // find string end
      if (len == 0) continue; // skip if empty
      memcpy(pkt + 1, payload, len); // load string
      esp_now_send(UGV_MAC, pkt, 1 + len); // blast it out
    }
  }
}

// Background Worker: Shoves air data up the wire to the Jetson
void serialTxTask(void* pv) {
  CommandPayload c; TelemetryPayload t; uint8_t m[64]; // storage
  for (;;) { // infinite loop
    if (xQueueReceive(qCmdToSerial, &c, 0) == pdTRUE) { // check commands
      serial_send_frame(TYPE_CMD, (const uint8_t*)&c, (uint8_t)sizeof(c)); // ship it
    }
    if (xQueueReceive(qTelemToSerial, &t, 0) == pdTRUE) { // check status
      serial_send_frame(TYPE_TELEM, (const uint8_t*)&t, (uint8_t)sizeof(t)); // ship it
    }
    if (xQueueReceive(qMsgToSerial, m, 0) == pdTRUE) { // check text
      uint8_t len = 0; while (len < 64 && m[len] != 0) len++; // find end
      if (len > 0) serial_send_frame(TYPE_MSG, m, len); // ship it
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // relax the cpu for 5ms
  }
}

// initial setup when the power turns on
void setup() {
  Serial.begin(115200); // start the data connection to jetson
  delay(500); // wait for voltages to settle

  WiFi.mode(WIFI_STA); // put the card in station mode for espnow
  if (esp_now_init() != ESP_OK) { // wake up the radio driver
    Serial.println("Error initializing ESP-NOW"); // failed
    while(true) delay(1000); // freeze here
  }

  // hand the listener function to the radio controller
  esp_now_register_recv_cb(onDataRecv);

  // tell the radio who we are allowed to talk to
  esp_now_peer_info_t peer = {}; // empty definition
  memcpy(peer.peer_addr, UGV_MAC, 6); // set ugv address
  peer.channel = 0; peer.encrypt = false; // defaults
  esp_now_add_peer(&peer); // register this peer

  // create the multi-worker lock
  serialMutex = xSemaphoreCreateMutex();

  // physically create the data buffers
  qTelemToNow  = xQueueCreate(10, sizeof(TelemetryPayload)); // 10 deep box
  qCmdToNow    = xQueueCreate(10, sizeof(CommandPayload)); // 10 deep box
  qMsgToNow    = xQueueCreate(10, 64); // string box
  qTelemToSerial = xQueueCreate(10, sizeof(TelemetryPayload)); // air status box
  qCmdToSerial = xQueueCreate(10, sizeof(CommandPayload)); // air cmd box
  qMsgToSerial = xQueueCreate(10, 64); // air msg box

  // spawn the workers into their own threads
  xTaskCreate(serialRxTask,      "SerRx",   4096, nullptr, 2, nullptr); // watching usb
  xTaskCreate(radioTxTelemTask,  "RadTelem",4096, nullptr, 2, nullptr); // radio ship status
  xTaskCreate(radioTxCmdTask,    "RadCmd",  4096, nullptr, 2, nullptr); // radio ship commands
  xTaskCreate(radioTxMsgTask,    "RadMsg",  4096, nullptr, 2, nullptr); // radio ship text
  xTaskCreate(serialTxTask,      "SerTx",   4096, nullptr, 1, nullptr); // shipping everything to jetson

  Serial.println("===================================="); // pretty header
  Serial.print("UAV Bridge Ready! MAC: "); // identification
  Serial.println(WiFi.macAddress()); // print mac for debugging
  Serial.println("===================================="); // footer
}

// main loop is empty because the workers handle everything
void loop() {
  vTaskDelay(portMAX_DELAY); // sleep forever and let workers do chores
}
