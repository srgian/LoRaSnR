#include <LoRa.h>
#include "boards.h"

#define VERSION "0.0.1"
#define DEVICES 4
#define MAX_TTL 10
#define MAX_HOP DEVICES - 1
#define MAX_BUF_SIZE 255

typedef struct
{
  uint8_t id;
  uint8_t ttl : 4;
  uint8_t hop : 4;
  uint16_t counter;
  int rssi;
  float snr;
  String toString() {
    char buff[256];
    sprintf(buff, "id: %u, ttl: %u, hop: %u, counter: %u, rssi: %d, snr: %f", id, ttl, hop, counter, rssi, snr);
    return String(buff);
  }
} device_t;

device_t devices[DEVICES];
device_t* me = devices;

int counter = 0;
uint8_t mac = 0;  //lsb (uint64_t % 255)

void setup() {
  initBoard();
  // When the power is turned on, a delay is required.
  delay(1500);
  /* TODO: better unique device is necessary to avoid ID=0 and colisions.
     * ID=0 means dead device
     */
  //mac = ESP.getEfuseMac();
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  mac = baseMac[5];
  memset(devices, 0, sizeof(devices));
  Serial.printf("LoRa Transceiver %s (%u)\n", VERSION, mac);
  Serial.printf("Size of devices = %d (%d * %d)\n", sizeof(devices), sizeof(device_t), DEVICES);

  if (mac == 0) {
    Serial.printf("WARNING!!! : MAC == 0\n");
  }

  if (sizeof(devices) > MAX_BUF_SIZE) {
    Serial.printf("WARNING!!! : Size of devices (%d) exceeds MAX_BUF_SIZE (%d)\n", sizeof(devices), MAX_BUF_SIZE);
  }

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if (!LoRa.begin(LoRa_frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();
}

void sendPacket() {
  /* copy alive devices in output buffer
     * first in buffer should always be own device
     * always alive (ID != 0, TTL<=MAX_TTL)
     * and transmissible (HOP<=MAX_HOP)
     */

  for (int i = 0; i < DEVICES; i++) {
    device_t* p = &(devices[i]);
    if (p->id != NULL && p->hop <= MAX_HOP) {
      Serial.printf("Sending: %s\n", p->toString().c_str());
      LoRa.beginPacket();
      LoRa.write((uint8_t*)p, sizeof(device_t));
      LoRa.endPacket();
    }
  }
}

void onReceive(int packetSize) {
  if (packetSize) {
    // received a packet
    uint8_t buf[256];
    uint8_t buf_idx = 0;
    device_t dev;

    uint8_t recv;
    while (LoRa.available()) {
      recv = (char)LoRa.read();
      buf[buf_idx++] = recv;
      if (buf_idx == sizeof(device_t)) {
        buf_idx = 0;
        memcpy((void*)(&dev), buf, sizeof(dev));
        dev.rssi = LoRa.packetRssi();
        dev.snr = LoRa.packetSnr();
        dev.hop++;
      }
    }

    int foundEmpty = 0;
    int foundID = 0;
    for (int i = 0; i < DEVICES; i++) {
      if (devices[i].id == NULL) {
        if (foundEmpty == 0) {
          foundEmpty = i;
        }
      }

      if (devices[i].id == dev.id) {
        if (devices[i].hop >= dev.hop) {
          foundID = i;
        } else {
          foundEmpty = 0;
          foundID = 0;
          break;
        }
      }
    }

    if (foundID != 0) {
      devices[foundID] = dev;
    } else if (foundEmpty != 0) {
      devices[foundEmpty] = dev;
    }
    // Serial.printf("Received: %s\n", dev.toString().c_str());
  }
}

void refreshDisplay() {
#ifdef HAS_DISPLAY
  if (u8g2) {
    char buf[256];
    u8g2->clearBuffer();
    // u8g2->drawStr(0, 12, "Transmitting: OK!");
    // snprintf(buf, sizeof(buf), "Sending: %d", counter);
    // u8g2->drawStr(0, 30, buf);
    snprintf(buf, sizeof(buf), "id ttl hop cnt rssi snr");
    u8g2->drawStr(0, 12, buf);
    for (int i = 0; i < DEVICES; i++) {
      snprintf(buf, sizeof(buf), "%3u %2u %2u %5u %+4d %+4.1f", devices[i].id, devices[i].ttl, devices[i].hop, devices[i].counter, devices[i].rssi, devices[i].snr);
      u8g2->drawStr(0, 24+i*12, buf);
    }
    u8g2->sendBuffer();
  }
#endif
}

void updateDevices() {
  int x = 0;
  Serial.printf("Devices:\n");
  for (int i = 0; i < DEVICES; i++) {
    device_t* p = &(devices[i]);
    /* adjust self */
    if (p == me) {
      me->id = mac;
      me->ttl = 0x00;
      me->hop = 0;
      me->counter = counter;
      me->rssi = 0;
      me->snr = 0.0f;
    } else {
      /* free device slot from devices for expired time to live
         * each device will set own ttl to 0 and will be transmited
         * to peers. each receiver should increase age of devices until
         * ttl expires. on receiveing fresh device data, TTL will be
         * adjusted to fresh data value, keeping the device info alive
         * device ID = 0 is considered killed.
         * Setup check is made that UID%uint8_t is not 0
         */
      if (p->ttl >= MAX_TTL) {
        // kill device
        p->id = 0;
        p->ttl = 0;
        p->hop = 0;
        p->counter = 0;
        p->rssi = 0;
        p->snr = 0.00;
      } else {
        // age device
        p->ttl++;
      }
    }
    if (p->id == NULL) {
      continue;
    }
    Serial.printf("    [%d]: %s\n", x, p->toString().c_str());
    x++;
  }
}

void loop() {
  Serial.println("");
  updateDevices();
  sendPacket();
  LoRa.receive();
  refreshDisplay();
  counter++;
  delay(1000);
}