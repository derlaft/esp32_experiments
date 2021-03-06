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
// 18:FE:34:FD:97:B2 (send)
// 18:FE:34:FD:98:74 (recv)

// #define BOARD_B2
#define BOARD_74


#define DEBUG_MAC
#define DEBUG_SEND
#define DEBUG_RECV
#define DEBUG_INIT
#define DEBUG_PEER_ADD
#define DEBUG_STOP

#define WIFI_CHANNEL 1
#define SERIAL_READ_TIMEOUT 1000
#define LOOP_INTERVAL 25
#define STATE_TIMEOUT 50

#if defined(BOARD_B2)
uint8_t peer_addr[] = {0x18, 0xFE, 0x34, 0xFD, 0x98, 0x74};
#define ROLE_SEND

#elif defined(BOARD_74)
uint8_t peer_addr[] = {0x18, 0xFE, 0x34, 0xFD, 0x97, 0xB2};
#define ROLE_RECV

#endif

#ifdef ROLE_SEND

#define BUTTON_UP 5
#define BUTTON_DOWN 4
#define BUTTON_LEFT 14
#define BUTTON_RIGHT 12
#define BUTTON_ACTION 13

#else

#define OUT_UP 5
#define OUT_DOWN 4
#define OUT_LEFT 14
#define OUT_RIGHT 12
#define OUT_ACTION 13
#define STOP_LEFT 2
#define STOP_RIGHT 16

unsigned long last_message_at;

#endif

typedef struct struct_message {
    bool left;
    bool right;
    bool up;
    bool down;
    bool action;
} struct_message;

#if defined(DEBUG_SEND) && defined(ROLE_SEND)
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

struct_message state;
struct_message new_state;

#ifdef ROLE_RECV
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

    if (sizeof(new_state) != len) {
#if defined(DEBUG_RECV)
        Serial.print("Unexpected bytes received: ");
        Serial.println(len);
        return;
#endif
    }

    memcpy(&new_state, incoming, sizeof(state));

#if defined(DEBUG_RECV)
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("up:");
    Serial.println(new_state.up);
    Serial.print("down:");
    Serial.println(new_state.down);
    Serial.print("left:");
    Serial.println(new_state.left);
    Serial.print("right:");
    Serial.println(new_state.right);
    Serial.print("action:");
    Serial.println(new_state.action);
#endif

    last_message_at = millis();
}
#endif

void setup(){

    state.up = false;
    state.down = false;
    state.left = false;
    state.right = false;
    state.action = false;

#ifdef ROLE_SEND
    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);
    pinMode(BUTTON_ACTION, INPUT_PULLUP);
#else
    pinMode(OUT_UP, OUTPUT);
    pinMode(OUT_DOWN, OUTPUT);
    pinMode(OUT_LEFT, OUTPUT);
    pinMode(OUT_RIGHT, OUTPUT);
    pinMode(OUT_ACTION, OUTPUT);

    pinMode(STOP_LEFT, INPUT_PULLUP);
    pinMode(STOP_RIGHT, INPUT_PULLUP);
#endif

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
#ifdef ROLE_SEND
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
#if defined(DEBUG_SEND)
    esp_now_register_send_cb(OnDataSent);
#endif
#elif defined(ROLE_RECV)
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
#endif
#endif

#ifdef ROLE_RECV
    esp_now_register_recv_cb(OnDataRecv);
#endif

    // тут же устанавливается канал и ключи шифрования (не устанавливатся)
    // канал очевидно должен совпадать

#if defined(ARDUINO_ARCH_ESP8266)
#ifdef ROLE_SEND
    esp_now_add_peer(peer_addr, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
#else
    esp_now_add_peer(peer_addr, ESP_NOW_ROLE_CONTROLLER, WIFI_CHANNEL, NULL, 0);
#endif
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

#ifdef ROLE_SEND
    // прочитать все пины
    new_state.up = !bool(digitalRead(BUTTON_UP));
    new_state.down = !bool(digitalRead(BUTTON_DOWN));
    new_state.left = !bool(digitalRead(BUTTON_LEFT));
    new_state.right = !bool(digitalRead(BUTTON_RIGHT));
    new_state.action = !bool(digitalRead(BUTTON_ACTION));

    bool any_active = 
        new_state.up ||
        new_state.down ||
        new_state.left ||
        new_state.right ||
        new_state.action;

    bool state_changed = 
        state.up != new_state.up ||
        state.down != new_state.down ||
        state.left != new_state.left ||
        state.right != new_state.right ||
        state.action != new_state.action;

    // посмотреть, изменилось ли что-то
    if (any_active || state_changed) {
        // отправить
        esp_now_send(peer_addr, (uint8_t *) &new_state, sizeof(new_state));
        // пометить отправленным
        memcpy(&state, &new_state, sizeof(state));
    }
#else

    if (new_state.left && new_state.right) {
        new_state.left = false;
        new_state.right = false;
#ifdef DEBUG_STOP
            Serial.println("left and right are active at the same time, preventing that");
#endif
    }

    if (new_state.up && new_state.down) {
        new_state.up = false;
        new_state.down = false;
#ifdef DEBUG_STOP
            Serial.println("up and down are active at the same time, preventing that");
#endif
    }

    bool is_stop_left = !digitalRead(STOP_LEFT);
    bool is_stop_right = !digitalRead(STOP_RIGHT);

    if (new_state.left && is_stop_left) {
#ifdef DEBUG_STOP
        Serial.println("stop_left active");
#endif
        new_state.left = false;
    }

    if (new_state.right && is_stop_right) {
#ifdef DEBUG_STOP
        Serial.println("stop_right active");
#endif
        new_state.right = false;
    }

    bool is_timeout = (millis() - last_message_at) > STATE_TIMEOUT;

    // timeout? stops?
    // cбросить пины
    if (is_timeout) {
        digitalWrite(OUT_UP, LOW);
        digitalWrite(OUT_DOWN, LOW);
        digitalWrite(OUT_LEFT, LOW);
        digitalWrite(OUT_RIGHT, LOW);
        digitalWrite(OUT_ACTION, LOW);
    } else {
        // установить пины
        if (new_state.up != state.up) {
            digitalWrite(OUT_UP, new_state.up);
        }
        if (new_state.down != state.down) {
            digitalWrite(OUT_DOWN, new_state.down);
        }
        if (new_state.left != state.left) {
            digitalWrite(OUT_LEFT, new_state.left && !is_stop_left);
        }
        if (new_state.right != state.right) {
            digitalWrite(OUT_RIGHT, new_state.right && !is_stop_right);
        }
        if (new_state.action != state.action) {
            digitalWrite(OUT_ACTION, new_state.action);
        }
        // пометить установленными
        memcpy(&state, &new_state, sizeof(state));
    }

#endif

    delay(LOOP_INTERVAL);
}
