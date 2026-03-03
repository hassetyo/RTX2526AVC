/**
 * client.cpp  —  UGV-side ESP32 bridge
 * ------------------------------------
 * Receives JSON packets from the UAV ESP32 via ESP-NOW
 * and forwards them to the Raspberry Pi 5 over UART.
 *
 * Responsibilities:
 *   • Initialize ESP-NOW receiver
 *   • Validate sender MAC to reject stray packets
 *   • Forward JSON lines to RPi5 (rpi_ugv_navigator.py)
 *
 * JSON format forwarded (newline-terminated):
 *   {"x":<float>,"y":<float>,"z":<float>,"vx":<float>,"vy":<float>,"t":<int>}
 *
 * Wiring (Serial2):
 *   ESP32 pin 4 (TX) --> RPi5 GPIO 15 / pin 10 (RX)  /dev/ttyAMA0
 *   ESP32 pin 5 (RX) <-- RPi5 GPIO 14 / pin  8 (TX)
 *   GND shared
 *
 * CHANGES FROM ORIGINAL:
 *   - Corrected UAV MAC comment to match actual server.cpp MAC
 *   - Added println newline for reliable Python readline() parsing
 *   - Removed misleading FORWARD_DELAY_MS — forwarding is event-driven
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define SERIAL_BAUD 115200

// Set this to the MAC address printed by server.cpp at boot.
// Example: if server says "MAC: C8:F0:9E:2C:15:00", enter that here.
static const uint8_t ESPNOW_SENDER_MAC[6] = { 0xC8, 0xF0, 0x9E, 0x2C, 0x15, 0x00 };

// ── Receive callback ──────────────────────────────────────────────────────────
void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {

  // Validate source to reject other ESP-NOW devices in range
  if (memcmp(mac_addr, ESPNOW_SENDER_MAC, 6) != 0) {
    Serial.println("[WARN] Packet from unknown MAC — ignored.");
    return;
  }

  // Null-terminate and convert to String
  char buf[len + 1];
  memcpy(buf, incomingData, len);
  buf[len] = '\0';
  String json = String(buf);

  // Debug print to USB serial monitor
  Serial.print("[RX] ");
  Serial.println(json);

  // Forward to Raspberry Pi 5 via UART.
  // println() appends '\n' which Python's readline() requires.
  Serial2.println(json);
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);           // USB debug
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, 4, 5);  // TX=GPIO4, RX=GPIO5 → RPi5
  delay(1000);

  WiFi.mode(WIFI_STA);
  Serial.println("[INFO] Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC:  " + WiFi.macAddress());  // <-- copy this to server.cpp UGV_MAC

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed — rebooting in 5 s...");
    delay(5000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataReceive);
  Serial.println("[INFO] ESP-NOW client ready — waiting for UAV packets...");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // All work is done in the receive callback.
  // Small delay to yield the CPU; does not affect callback timing.
  delay(100);
}
