#include <Arduino.h>


#if defined(ARDUINO_ARCH_ESP8266)

#define SERIAL_SPEED 74880

#include <ESP8266WiFi.h>
#include <espnow.h>

// отсутствующие константы в esp8266-версии
#ifndef ESP_NOW_SEND_SUCCESS
#define ESP_NOW_SEND_SUCCESS (0)
#endif
#ifndef ESP_OK
#define ESP_OK (0)
#endif

#elif defined(ARDUINO_ARCH_ESP32)

#define SERIAL_SPEED 115200

#include <WiFi.h>
#include <esp_now.h>

#endif


// esp8266 boards (talk to each other):
// 2C:3A:E8:05:EB:C4
// 2C:3A:E8:05:EA:82
//
// esp32 boards (talk to each other):
// 80:7D:3A:C5:6C:08
// 30:AE:A4:C1:57:30

// #define BOARD_82
// #define BOARD_C4
#define BOARD_30
// #define BOARD_08

// #define DEBUG_MAC
// #define DEBUG_SEND
// #define DEBUG_RECV
// #define DEBUG_INIT
// #define DEBUG_PEER_ADD

#define WIFI_CHANNEL 1
#define SERIAL_READ_TIMEOUT 20

#if defined(BOARD_82)
uint8_t peer_addr[] = {0x2C, 0x3A, 0xE8, 0x05, 0xEB, 0xC4};
#elif defined(BOARD_C4)
uint8_t peer_addr[] = {0x2C, 0x3A, 0xE8, 0x05, 0xEA, 0x82};
#elif defined(BOARD_08)
uint8_t peer_addr[] = {0x30, 0xAE, 0xA4, 0xC1, 0x57, 0x30};
#elif defined(BOARD_30)
uint8_t peer_addr[] = {0x80, 0x7D, 0x3A, 0xC5, 0x6C, 0x08};
#endif

#if defined(DEBUG_SEND)
void OnDataSent(
#if defined(ARDUINO_ARCH_ESP8266)
        uint8_t *mac_addr,
        uint8_t sendStatus
#elif defined(ARDUINO_ARCH_ESP32)
        const uint8_t *mac_addr,
        esp_now_send_status_t sendStatus
#endif
) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == ESP_NOW_SEND_SUCCESS){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
#endif

void OnDataRecv(
#if defined(ARDUINO_ARCH_ESP8266)
        uint8_t *mac,
        uint8_t *incoming,
        uint8_t len
#elif defined(ARDUINO_ARCH_ESP32)
        const uint8_t *mac,
        const uint8_t *incoming,
        int len
#endif
) {

#if defined(DEBUG_RECV)
    Serial.print("Bytes received: ");
    Serial.println(len);
#endif

    Serial.write(incoming, len);
}

void setup(){
    Serial.begin(SERIAL_SPEED);
    Serial.setTimeout(SERIAL_READ_TIMEOUT);

    WiFi.mode(WIFI_STA);

    // нопечатать mac-адрес
#if defined(DEBUG_MAC)
    Serial.println();
    Serial.print("Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
#endif

    // инициализация esp now
    if (esp_now_init() != ESP_OK) {
#ifdef DEBUG_INIT
        Serial.println("Error initializing ESP-NOW");
#endif
        return;
    }

#if defined(ARDUINO_ARCH_ESP8266)
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
#endif

#if defined(DEBUG_SEND)
    esp_now_register_send_cb(OnDataSent);
#endif

    esp_now_register_recv_cb(OnDataRecv);

    // тут же устанавливается канал и ключи шифрования (не устанавливатся)
    // канал очевидно должен совпадать

#if defined(ARDUINO_ARCH_ESP8266)
    esp_now_add_peer(peer_addr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
#elif defined(ARDUINO_ARCH_ESP32)
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, peer_addr, 6);
    peerInfo.channel = WIFI_CHANNEL;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
#if defined(DEBUG_PEER_ADD)
        Serial.println("Failed to add peer");
#endif
        return;
    }
#endif
}
 
void loop(){

    uint8_t buf[250];
    // TODO: возможно нужно занулить buf

    // читаем из serial до 250 байт
    size_t len = Serial.readBytesUntil('\n', buf, 250);
    if (len > 0) {
          // отправляем peer
          esp_now_send(peer_addr, (uint8_t *) &buf, len);
    }
}
