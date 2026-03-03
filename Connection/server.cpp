/**
 * server.cpp  —  UAV-side ESP32 bridge
 * ------------------------------------
 * Forwards JSON data from the Jetson to the UGV via ESP-NOW.
 *
 * Responsibilities:
 *   • Read JSON lines from Jetson UART (USB or GPIO serial)
 *   • Send those packets directly to paired ESP32 on UGV
 *   • Print acknowledgments for debugging
 *
 * JSON packet format produced by jetson_uav_sender.py:
 *   {"x":<float>,"y":<float>,"z":<float>,"vx":<float>,"vy":<float>,"t":<int>}
 *
 * JSON packets are newline-terminated and must not be reformatted.
 * Transmission pacing matches client.cpp receive rate (500 ms).
 *
 * CHANGES FROM ORIGINAL:
 *   - Fixed typo "Reeboting" -> "Rebooting"
 *   - Increased buffer guard to 250 bytes (ESP-NOW max is 250)
 */

#include <WiFi.h>
#include <esp_now.h>

#define SERIAL_BAUD    115200
#define SERIAL_TIMEOUT 50          // ms readline timeout
#define ESPNOW_MAX     250         // ESP-NOW hard payload limit (bytes)

// Replace with your actual UGV ESP32 MAC address.
// Run client.cpp once and read "MAC Address: xx:xx:xx:xx:xx:xx" from Serial Monitor.
const uint8_t UGV_MAC[] = {0xC8, 0x2E, 0x18, 0xFB, 0xCC, 0x70};

String buffer = "";

// ── ESP-NOW send callback ─────────────────────────────────────────────────────
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMEOUT);
  delay(1000);

  WiFi.mode(WIFI_STA);
  Serial.println("[INFO] Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC:  " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed — rebooting in 5 s...");
    delay(5000);
    ESP.restart();
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, UGV_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add peer — halting.");
    while (true) { delay(1000); }
  }

  esp_now_register_send_cb(onSend);
  Serial.println("[INFO] ESP-NOW ready — waiting for Jetson JSON...");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    buffer = Serial.readStringUntil('\n');
    buffer.trim();

    if (buffer.length() == 0) return;

    // Guard against ESP-NOW 250-byte limit
    if (buffer.length() > ESPNOW_MAX) {
      Serial.printf("[WARN] Packet too large (%d bytes) — dropping.\n", buffer.length());
      return;
    }

    esp_err_t result = esp_now_send(UGV_MAC,
                                    (uint8_t *)buffer.c_str(),
                                    buffer.length());
    if (result == ESP_OK) {
      Serial.print("[TX] ");
      Serial.println(buffer);
    } else {
      Serial.println("[ERROR] esp_now_send failed!");
    }

    // Pacing: matches the 500 ms delay expected by the UGV client.
    // Also keeps within the 2 Hz stream rate from jetson_uav_sender.py.
    delay(500);
  }
}
