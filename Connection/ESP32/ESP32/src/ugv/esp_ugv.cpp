#include <Arduino.h> // basics like setup/loop/serial
#include <WiFi.h> // radio mode control for espnow hardware
#include <esp_now.h> // radio api for sending raw packets over air

#include "freertos/FreeRTOS.h" // foundation for the task switching logic
#include "freertos/task.h" // lets me hire workers for background chores
#include "freertos/queue.h" // mailboxes for safe cross-task talking

// mac address of esp32_1 (UAV) = F8:B3:B7:20:25:A8
// mac address of esp32_2 (UGV) = F8:B3:B7:20:69:C0

// UGV receives telemetry from UAV and sends commands to UAV

/////////////////////////////////config stuff
// giving the ugv the mac adress of UAV as raw bytes
// so the ESP-NOW targets devices by MAC address
// we need to manually add the peer (UAV) to the ESP-NOW network
static uint8_t UAV_MAC[6] = {0xf8, 0xb3, 0xb7, 0x20, 0x25, 0xa8}; 

//////////////////// protocol stuff
/* i had to add protocol cause , when i was sending raw bytes over usb, sometimes a byte got lost or corrupted
 a protocol in this case is like a set of grammar rules. if the message starts with a specific "start" byte and ends with a 
 "checksum" we can prove that the data is 100% correct before using it.
*/
// sof (start of frame). m using 0xAA (10101010) in binary
static const uint8_t SOF = 0xAA; // marker for start of packet
static const uint8_t TYPE_TELEM = 1; // code for sensor data packets
static const uint8_t TYPE_CMD   = 2; // code for mission commands
static const uint8_t TYPE_MSG   = 3; // code for raw debug strings

//////////////////////////// STRUCTS

// making the c struct packet definitions
// so the compiler doesnt add any padding bytes 
typedef struct __attribute__((packed)) {
  uint32_t seq;    // counts up to track lost packets
  uint32_t t_ms;   // internal clock time in ms
  float vx;        // velocity x
  float vy;        // velocity y
  uint8_t marker;  // detection flag
  uint8_t estop;   // mission abort flag
} TelemetryPayload;

typedef struct __attribute__((packed)) {
  uint32_t cmdSeq; // unique move id
  uint8_t  cmd;    // 1=arm, 2=disarm, etc
  uint8_t  estop;  // emergency stop flag
} CommandPayload;

//////////////////// QUEUES (Mailboxes)
// i had to add these to pass data between the serial task (USB) and the radio tasks
static QueueHandle_t qTelemToSerial = nullptr; // Radio data ready for USB
static QueueHandle_t qCmdToSerial   = nullptr; // Commands ready for USB
static QueueHandle_t qMsgToSerial   = nullptr; // Text ready for USB
static QueueHandle_t qTelemToNow    = nullptr; // Pi data ready for Radio
static QueueHandle_t qCmdToNow      = nullptr; // Pi commands ready for Radio
static QueueHandle_t qMsgToNow      = nullptr; // Pi text ready for Radio

static SemaphoreHandle_t serialMutex = nullptr; // lock to stop workers talking over each other

// i had to add this casue , needed a way to seal them so i know they werent tampered with
static uint8_t checksum_xor(uint8_t type, uint8_t len, const uint8_t* payload) {
  uint8_t c = type ^ len; // initial seed from header
  for (uint8_t i = 0; i < len; i++) c ^= payload[i]; // mix in all data
  return c; // final signature
}

// wrapping data "seal the envelope" for serial transmission
// lmao whoever is debugging my code good luck! discord me on : catari_express
static void serial_send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t chk = checksum_xor(type, len, payload); // get the seal
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) { // grab the lock
    Serial.write(SOF);      // 1. send start marker
    Serial.write(type);     // 2. send packet type
    Serial.write(len);      // 3. send size
    if (len > 0) Serial.write(payload, len); // 4. send actual memory
    Serial.write(chk);      // 5. send checksum seal
    xSemaphoreGive(serialMutex); // release the lock
  }
}

//////////////////// ESP-NOW RECIEVE CALLBACK
// this runs automatically when a radio packet arrives from the UAV
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 1) return; // skip if size is zero
  uint8_t fType = data[0]; // first byte is always message type

  // radio echo (debug): tell the RPi we got SOMETHING over the air
  char dbg[64]; // temporary string buffer
  snprintf(dbg, 64, "ESP: Got Radio PKT type %d len %d", fType, len); // format text
  if (qMsgToSerial) { // if mailbox is ready
    uint8_t dBuf[64]; // byte array
    memset(dBuf, 0, 64); // clean memory
    memcpy(dBuf, dbg, strlen(dbg)); // copy text
    xQueueSend(qMsgToSerial, dBuf, 0); // ship to usb worker
  }

  // Figure out what the UAV said and put it in the mailbox for the USB task
  if (fType == TYPE_TELEM && len >= (int)sizeof(TelemetryPayload)) {
    TelemetryPayload t; memcpy(&t, data + 1, sizeof(t)); // unpack status
    if (qTelemToSerial) xQueueSend(qTelemToSerial, &t, 0); // ship to usb worker
  }
  else if (fType == TYPE_CMD && len >= (int)sizeof(CommandPayload)) {
    CommandPayload cmd; memcpy(&cmd, data + 1, sizeof(cmd)); // unpack command
    if (qCmdToSerial) xQueueSend(qCmdToSerial, &cmd, 0); // ship to usb worker
  }
  else if (fType == TYPE_MSG) {
    uint8_t payload[64]; memset(payload, 0, 64); // unpack text
    uint8_t msgLen = (len - 1 > 64) ? 64 : len - 1; // clip length
    memcpy(payload, data+1, msgLen); // extract string
    if (qMsgToSerial) xQueueSend(qMsgToSerial, payload, 0); // ship to usb worker
  }
}

//////////////////// PARSER (Gatekeeper)
// stages for the state machine to unpack USB data
enum ParseState { WAIT_SOF, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHK };
typedef struct {
  ParseState st; // current mode
  uint8_t type; // header type
  uint8_t len; // payload size
  uint8_t idx; // packet counter
  uint8_t buf[64]; // holding area
} SerialParser;

// resets the parser to hunt for the lead 0xAA byte
static void parser_init(SerialParser* p) { p->st = WAIT_SOF; }

// feeds one byte at a time through the grammar rules
static bool parser_step(SerialParser* p, uint8_t b, uint8_t* outType, uint8_t* outLen, uint8_t* outPayload) {
  switch (p->st) {
    case WAIT_SOF: if (b == SOF) p->st = WAIT_TYPE; break; // hunt for 0xAA
    case WAIT_TYPE: p->type = b; p->st = WAIT_LEN; break; // save label
    case WAIT_LEN:
      p->len = b; // save size
      if (p->len > 64) { parser_init(p); break; } // reset if too big
      p->idx = 0; p->st = (p->len == 0) ? WAIT_CHK : WAIT_PAYLOAD; // move forward
      break;
    case WAIT_PAYLOAD:
      p->buf[p->idx++] = b; // collect data
      if (p->idx >= p->len) p->st = WAIT_CHK; // got it all
      break;
    case WAIT_CHK: {
      uint8_t expected = checksum_xor(p->type, p->len, p->buf); // check seal
      if (b == expected) { // match!
        *outType = p->type; *outLen = p->len; // export info
        memcpy(outPayload, p->buf, p->len); // export content
        parser_init(p); return true; // message is clean
      }
      parser_init(p); break; // failed signature, restart
    }
  }
  return false; // still looking
}

//////////////////// TASKS (Workers)

// Worker 1: Takes Telemetry from mailbox and sends to RPi (USB)
void serialTxTelemTask(void* pv) {
  TelemetryPayload t; // create storage
  for (;;) { // loop forever
    if (xQueueReceive(qTelemToSerial, &t, portMAX_DELAY) == pdTRUE) { // wait for status
      serial_send_frame(TYPE_TELEM, (const uint8_t*)&t, (uint8_t)sizeof(t)); // ship it
    }
  }
}

// Worker 2: Takes Commands from Air and sends to RPi (USB)
void serialTxCmdTask(void* pv) {
  CommandPayload c; // create storage
  for (;;) { // loop forever
    if (xQueueReceive(qCmdToSerial, &c, portMAX_DELAY) == pdTRUE) { // wait for command
      serial_send_frame(TYPE_CMD, (const uint8_t*)&c, (uint8_t)sizeof(c)); // ship it
    }
  }
}

// Worker 3: Sends Messages from UAV to RPi (USB)
void serialTxMsgTask(void* pv) {
  uint8_t payload[64]; // create text buffer
  for (;;) { // loop forever
    if (xQueueReceive(qMsgToSerial, payload, portMAX_DELAY) == pdTRUE) { // wait for text
      uint8_t len = 0; while (len < 64 && payload[len] != 0) len++; // find end of string
      if (len == 0) continue; // skip if empty
      serial_send_frame(TYPE_MSG, payload, len); // ship it
    }
  }
}

// Worker 4: Listens to the RPi for status or commands to send over Radio
void serialRxTask(void* pv) {
  SerialParser parser; // create gatekeeper
  parser_init(&parser); // reset state
  uint8_t type, len, payload[64]; // temporary variables
  for (;;) { // loop forever
    while (Serial.available() > 0) { // check usb buffer
      uint8_t b = (uint8_t)Serial.read(); // pull one byte
      if (parser_step(&parser, b, &type, &len, payload)) { // check rules
        if (type == TYPE_CMD && len == sizeof(CommandPayload)) {
          CommandPayload c; memcpy(&c, payload, sizeof(c)); // unpack
          if (qCmdToNow) xQueueSend(qCmdToNow, &c, 0); // ship to radio task
        }
        else if (type == TYPE_TELEM && len == sizeof(TelemetryPayload)) {
          TelemetryPayload t; memcpy(&t, payload, sizeof(t)); // unpack
          if (qTelemToNow) xQueueSend(qTelemToNow, &t, 0); // ship to radio task
        }
        else if (type == TYPE_MSG) {
          uint8_t msgBuf[64]; memset(msgBuf, 0, 64); // unpack text
          memcpy(msgBuf, payload, len); // extraction
          if (qMsgToNow) xQueueSend(qMsgToNow, msgBuf, 0); // ship to radio task
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // relax the cpu for 5ms
  }
}

// Worker 5: Master radio sender. Grabs stuff from mailboxes and throws it at the UAV over air
void espNowTxTask(void* pv) {
  CommandPayload c; TelemetryPayload t; uint8_t m[64]; // mailbox storage
  uint8_t pktCmd[1 + sizeof(CommandPayload)]; pktCmd[0] = TYPE_CMD; // packet template
  uint8_t pktTelem[1 + sizeof(TelemetryPayload)]; pktTelem[0] = TYPE_TELEM; // packet template
  uint8_t pktMsg[1 + 64]; pktMsg[0] = TYPE_MSG; // packet template

  for (;;) { // loop forever
    if (xQueueReceive(qCmdToNow, &c, 0) == pdTRUE) { // check for orders
      memcpy(pktCmd + 1, &c, sizeof(c)); // load command
      esp_now_send(UAV_MAC, pktCmd, sizeof(pktCmd)); // blast over air
    }
    if (xQueueReceive(qTelemToNow, &t, 0) == pdTRUE) { // check for status
      memcpy(pktTelem + 1, &t, sizeof(t)); // load status
      esp_now_send(UAV_MAC, pktTelem, sizeof(pktTelem)); // blast over air
    }
    if (xQueueReceive(qMsgToNow, m, 0) == pdTRUE) { // check for messages
      uint8_t mLen = 0; while (mLen < 64 && m[mLen] != 0) mLen++; // find string end
      if (mLen > 0) { // if text exists
        memcpy(pktMsg + 1, m, mLen); // load message
        esp_now_send(UAV_MAC, pktMsg, 1 + mLen); // blast over air
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // wait 5ms before next loop
  }
}

//////////////////// SETUP AND LOOP

// initial setup when power turns on
void setup() {
  Serial.begin(115200); // start usb connection to pi
  delay(500); // wait for hardware to settle

  WiFi.mode(WIFI_STA); // put card in station mode for radio driver
  if (esp_now_init() != ESP_OK) return; // start espnow

  esp_now_register_recv_cb(onDataRecv); // hand radio listener function

  // manually add the UAV as a peer
  esp_now_peer_info_t peer = {}; // empty definition
  memcpy(peer.peer_addr, UAV_MAC, 6); // set drone address
  peer.channel = 0; peer.encrypt = false; // defaults
  esp_now_add_peer(&peer); // register this drone

  // create the cross-worker lock
  serialMutex = xSemaphoreCreateMutex();

  // physical mailboxes for handoffs
  qTelemToSerial = xQueueCreate(10, sizeof(TelemetryPayload)); // 10 deep box
  qCmdToSerial   = xQueueCreate(10, sizeof(CommandPayload)); // 10 deep box
  qMsgToSerial   = xQueueCreate(10, 64); // string mailbox
  qTelemToNow    = xQueueCreate(10, sizeof(TelemetryPayload)); // outbound box
  qCmdToNow      = xQueueCreate(10, sizeof(CommandPayload)); // outbound box
  qMsgToNow      = xQueueCreate(10, 64); // outbound string box

  // hire the background workers in their own threads
  xTaskCreate(serialTxTelemTask, "TxTelem", 4096, NULL, 2, NULL); // shipping status to pi 
  xTaskCreate(serialTxCmdTask,   "TxCmd",   4096, NULL, 2, NULL); // shipping orders to pi
  xTaskCreate(serialTxMsgTask,   "TxMsg",   4096, NULL, 2, NULL); // shipping talk to pi
  xTaskCreate(serialRxTask,      "SerRx",   4096, NULL, 2, NULL); // watching pi for data
  xTaskCreate(espNowTxTask,      "NowTx",   4096, NULL, 2, NULL); // blasting air packets

  Serial.println("===================================="); // pretty header
  Serial.print("UGV Bridge Ready! MAC: "); // identifying this chip
  Serial.println(WiFi.macAddress()); // print mac for link verification
  Serial.println("===================================="); // footer
}

// freeRTOS tasks handle everything so main loop is silent
void loop() {
  vTaskDelay(portMAX_DELAY); // sleep forever and let workers do their thing
}
