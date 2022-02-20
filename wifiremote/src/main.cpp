#include <Arduino.h>

#ifdef TOUCH_UI
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h> 
#include <Adafruit_FT6206.h>
#include "esp_freertos_hooks.h"
#include "lvgl.h"
#endif

#if defined(ARDUINO_ARCH_ESP8266)

// скорость серийного порта
// совпадает со стандартной
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

// скорость серийного порта
// совпадает со стандартной
#define SERIAL_SPEED 115200

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#endif

// выводить отладочные сообщения, связанные с MAC-адресом
#define DEBUG_MAC
// выводить отладочные сообщения, связанные с отправкой пакетов
#define DEBUG_SEND
// выводить отладочные сообщения, связанные с получением пакетов
#define DEBUG_RECV
// выводить отладочные сообщения, связанные с инициализацией
#define DEBUG_INIT
// выводить отладочные сообщения, связанные с пейрингом устройств
#define DEBUG_PEER_ADD
// выводить отладочные сообщения, связанные с ENDSTOP
#define DEBUG_STOP
// выводить отладочные сообщения, связанные с тачскрином на WT32-SC01
#define DEBUG_TOUCH
// выводить отладочные сообщения, связанные с LVGL
// дополнительно необходимо включить LV_USE_LOG во флагах сборки
#define DEBUG_LVGL

// wifi-канал
#define WIFI_CHANNEL 1
// timeout чтения из серийного порта (вряд ли важно)
#define SERIAL_READ_TIMEOUT 1000
// задержка между итерациями главного цикла
#define LOOP_INTERVAL 25
// задержка, после которой считается, что сигнал потерян и сбрасывается состояние всего
#define STATE_TIMEOUT 50
// задержка, после которой можно двигать ось в обратном направлении
#define BLOCK_TIMEOUT 1000
// задержка, после которой уменьшается яркость экрана
#define SCREENSAVER_TIMEOUT 60000
// переодичность отправки сообщений о здоровье
#define HEALTH_INTERVAL 1000

// Платы esp8266 (говорят друг с другом):
// 18:FE:34:FD:97:B2 (send)
// 18:FE:34:FD:98:74 (recv)

// Если установлена эта настройка, MAC-адрес будет перезаписан
// Иначе (если выключено) нужно обязательно изменить адреса ниже
// Адреса можно узнать, включив DEBUG_MAC
#define CHANGE_MAC

// Включить автоматическое выключение экрана после периода неактивности
// #define SCREENSAVER

// выбор платы
// сейчас происходит в env platformio.ini
// #define BOARD_B2
// #define BOARD_74

uint8_t recv_addr[] = {0x18, 0xFE, 0x34, 0xFD, 0x98, 0x74};
uint8_t send_addr[] = {0x18, 0xFE, 0x34, 0xFD, 0x97, 0xB2};

#if defined(BOARD_B2)
#define PEER_ADDR recv_addr
#define SELF_ADDR send_addr
#define ROLE_SEND

#elif defined(BOARD_74)
#define PEER_ADDR send_addr
#define SELF_ADDR recv_addr
#define ROLE_RECV

#endif

#ifdef ROLE_SEND

#define BUTTON_UP 5
#define BUTTON_DOWN 4
#define BUTTON_LEFT 14
#define BUTTON_RIGHT 12
#define BUTTON_ACTION 13

#else

#if defined(ARDUINO_ARCH_ESP8266)
#define OUT_UP 5
#define OUT_DOWN 4
#define OUT_LEFT 14
#define OUT_RIGHT 12
#define OUT_ACTION 13
#define STOP_RIGHT 16
#define STOP_LEFT A0
#elif defined(ARDUINO_ARCH_ESP32)
#define OUT_UP 16
#define OUT_DOWN 17
#define OUT_LEFT 18
#define OUT_RIGHT 19
#define OUT_ACTION 21
#define STOP_RIGHT 22
#define STOP_LEFT 23
#endif

#define STOP_LEFT_THRESHOLD 500

unsigned long last_message_at;
unsigned long leftright_blocked_at;
unsigned long updown_blocked_at;

#endif

#ifdef REPORT_MESSAGES
unsigned long last_report_at;
#endif

typedef struct struct_message {
    bool left;
    bool right;
    bool up;
    bool down;
    bool action;
} struct_message;

enum report_message_type {
    health,
    on_boot,
};

typedef struct struct_report_message {
    report_message_type type;
} struct_report_message;

// общие переменные состояния
// используются и на RECV, и на SEND
struct_message state;
struct_message new_state;

// начало блока, связанного с пользовательским интерфейсом
#if defined(TOUCH_UI) && defined(ROLE_SEND)

#ifdef REPORT_MESSAGES
bool is_popup = false;
#endif

// список кнопок
static const char * btnm_map[] = {
    "_",
    LV_SYMBOL_UP,
    "_",
    "\n",
    LV_SYMBOL_LEFT,
    LV_SYMBOL_CHARGE,
    LV_SYMBOL_RIGHT,
    "\n",
    "_",
    LV_SYMBOL_DOWN,
    "_",
    "",
};

static const lv_btnmatrix_ctrl_t btnm_control[] = {
    LV_BTNMATRIX_CTRL_HIDDEN,
    0,
    LV_BTNMATRIX_CTRL_HIDDEN,
    0,
    0,
    0,
    LV_BTNMATRIX_CTRL_HIDDEN,
    0,
    LV_BTNMATRIX_CTRL_HIDDEN,
    0,
    0,
    0,
};

// список кнопок всплывающего сообщения
static const char * msgbox_buttons[] = {
    "Принято",
    ""
};

static const char * msgbox_title = "Сообщение от приёмника";

static const char * msgbox_messages[] = {
    "Приемник сообщил о загрузке.",
    ""
};

#ifdef REPORT_MESSAGES
static void msgbox_cb(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_current_target(e);
    lv_msgbox_close(obj);
    is_popup = false;
}
#endif

TFT_eSPI tft = TFT_eSPI();
Adafruit_FT6206 touchScreen = Adafruit_FT6206();


#ifdef SCREENSAVER
unsigned long last_touch_event_at;
uint8_t brightness = 255;
#endif

// массив кнопок
lv_obj_t * btnm;

#ifdef REPORT_MESSAGES
// индикатор состояния соединения
lv_obj_t * led;
#endif

// буфер отрисовки
static const uint16_t screenWidth  = TFT_HEIGHT;
static const uint16_t screenHeight = TFT_WIDTH;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

#ifdef DEBUG_LVGL
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
  static uint16_t lastx = 0;
  static uint16_t lasty = 0;

  if (!touchScreen.touched()) {
     data->state = LV_INDEV_STATE_REL;
     data->point.x = lastx;
     data->point.y = lasty;
     return;
  }

#ifdef SCREENSAVER
  last_touch_event_at = millis();

  if (brightness == LOW) {
      brightness = HIGH;
      digitalWrite(TFT_BL, brightness);
      // avoid generating an event - wakeup
      return;
  }
#endif

  TS_Point touchPos = touchScreen.getPoint();
  data->state = LV_INDEV_STATE_PR;

#ifdef DEBUG_TOUCH
  Serial.println("touch at " + String(touchPos.x) + "x" + String(touchPos.y));
#endif
  auto xpos = touchPos.y;
  auto ypos = TFT_WIDTH - touchPos.x;
#ifdef DEBUG_TOUCH
  Serial.println("mapped to " + String(xpos) + "x" + String(ypos));
#endif

  data->point.x = xpos;
  data->point.y = ypos;
  lastx = xpos;
  lasty = ypos;

  return;
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

static void lv_tick_task(void)
{
 lv_tick_inc(portTICK_RATE_MS);
}

// конец блока, связанного с пользовательским интерфейсом
#endif



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

#if defined(ROLE_RECV) || defined(REPORT_MESSAGES)

uint32_t delay_updown = 0;
uint32_t delay_leftright = 0;
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
#ifdef ROLE_RECV

    // логика приёма сообщения на приёмнике

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
#else

#ifdef REPORT_MESSAGES

    // логика приёма сообщения на пульте
    static struct_report_message got_msg;
    if (sizeof(got_msg) != len) {
#if defined(DEBUG_RECV)
        Serial.print("Unexpected bytes received: ");
        Serial.println(len);
        return;
#endif
    }

    // скопировать сообщение
    memcpy(&got_msg, incoming, sizeof(got_msg));
    last_report_at = millis();

#if defined(DEBUG_RECV)
        Serial.print("Received status message:");
        Serial.println(got_msg.type);
#endif

    switch (got_msg.type) {
        case health:
            break;
        case on_boot:
            // показать popup инициализации
            // только если уже не показан
            if (!is_popup) {
                lv_obj_t * mbox = lv_msgbox_create(NULL, msgbox_title, msgbox_messages[0], msgbox_buttons, false);
                lv_obj_add_event_cb(mbox, msgbox_cb, LV_EVENT_VALUE_CHANGED, NULL);
                lv_obj_set_style_text_font(mbox, &hack_14_cyr, 0);
                lv_obj_center(mbox);
                is_popup = true;
            }
            break;
    }

#endif

#endif
}

void setup(){

    memset(&state, 0, sizeof(state));
#ifdef ROLE_RECV
    leftright_blocked_at = millis();
    updown_blocked_at = millis();
#endif

#ifdef REPORT_MESSAGES
    last_report_at = millis();
#endif

#ifdef ROLE_SEND
#ifndef TOUCH_UI
    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);
    pinMode(BUTTON_ACTION, INPUT_PULLUP);
#endif
#else
    pinMode(OUT_UP, OUTPUT);
    pinMode(OUT_DOWN, OUTPUT);
    pinMode(OUT_LEFT, OUTPUT);
    pinMode(OUT_RIGHT, OUTPUT);
    pinMode(OUT_ACTION, OUTPUT);

    digitalWrite(OUT_UP, LOW);
    digitalWrite(OUT_DOWN, LOW);
    digitalWrite(OUT_LEFT, LOW);
    digitalWrite(OUT_RIGHT, LOW);
    digitalWrite(OUT_ACTION, LOW);

    pinMode(STOP_RIGHT, INPUT_PULLUP);
#ifdef ARDUINO_ARCH_ESP32
    pinMode(STOP_LEFT, INPUT_PULLUP);
#endif
#endif

    Serial.begin(SERIAL_SPEED);
    Serial.setTimeout(SERIAL_READ_TIMEOUT);

    WiFi.mode(WIFI_STA);

#if defined(CHANGE_MAC)
#if defined(ARDUINO_ARCH_ESP32)
  esp_wifi_set_mac(WIFI_IF_STA, &SELF_ADDR[0]);
#elif defined(ARDUINO_ARCH_ESP8266)
  wifi_set_macaddr(STATION_IF, &SELF_ADDR[0]);
#endif

#if defined(DEBUG_MAC)
  Serial.println("Changed MAC Adress");
#endif
#endif

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

#if defined(ROLE_RECV) || defined(REPORT_MESSAGES)
    esp_now_register_recv_cb(OnDataRecv);
#endif

    // тут же устанавливается канал и ключи шифрования (не устанавливатся)
    // канал очевидно должен совпадать

#if defined(ARDUINO_ARCH_ESP8266)
#ifdef ROLE_SEND
    esp_now_add_peer(PEER_ADDR, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
#else
    esp_now_add_peer(PEER_ADDR, ESP_NOW_ROLE_CONTROLLER, WIFI_CHANNEL, NULL, 0);
#endif
#elif defined(ARDUINO_ARCH_ESP32)
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, PEER_ADDR, 6);
    peerInfo.channel = WIFI_CHANNEL;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
#if defined(DEBUG_PEER_ADD)
        Serial.println("Failed to add peer");
#endif
        return;
    }
#endif

#if defined(TOUCH_UI) and defined(ROLE_SEND)
    esp_err_t err = esp_register_freertos_tick_hook((esp_freertos_tick_cb_t)lv_tick_task); 
    // TODO: check err

    // включить подсветку
#ifdef SCREENSAVER
    brightness = HIGH;
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, brightness);
#else
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
#endif

    // Start TouchScreen
    // requires custom I2C pinout
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    if (!touchScreen.begin(40)) { 
      Serial.println("Unable to start touchscreen.");
    }

    // Enable TFT
    tft.begin();
    tft.setRotation(1);

    // Init lvgl
    lv_init();
#if defined(DEBUG_LVGL) && LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );

    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /* Initialize the input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_input_read;
    lv_indev_drv_register( &indev_drv );

    // set root font
    lv_obj_set_style_text_font(lv_scr_act(), &hack_14_cyr, 0);

    // матрица кнопок для управления направлением
    btnm = lv_btnmatrix_create(lv_scr_act());
    lv_btnmatrix_set_map(btnm, btnm_map);
    lv_btnmatrix_set_ctrl_map(btnm, btnm_control);
    lv_obj_align(btnm, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(btnm, &lv_font_montserrat_40, 0);
    lv_obj_set_height(btnm, lv_pct(90));
    lv_obj_set_width(btnm, lv_pct(90));

    // индикатор состояния соединения
#ifdef REPORT_MESSAGES
    led  = lv_led_create(btnm);
    lv_obj_align(led, LV_ALIGN_TOP_RIGHT, -10, 0);
    lv_led_set_brightness(led, 150);
    lv_led_set_color(led, lv_palette_main(LV_PALETTE_GREEN));
    lv_led_off(led);
#endif


#endif

#if defined(ROLE_RECV) && defined(REPORT_MESSAGES)
    static struct_report_message send = {on_boot};
    esp_now_send(PEER_ADDR, (uint8_t *) &send, sizeof(send));
#ifdef DEBUG_SEND
    Serial.println("trying to send on-boot notification msg");
#endif

#endif

}
 
void loop(){

#ifdef ROLE_SEND

#ifdef TOUCH_UI

    // обнулить все перед чтением
    memset(&new_state, 0, sizeof(new_state));

    // прочитать состояние кнопки
    uint32_t id = lv_btnmatrix_get_selected_btn(btnm);
    switch (id) {
        case LV_BTNMATRIX_BTN_NONE:
            // никакая кнопка не нажата
            break;
        case 1:
            new_state.up = HIGH;
            break;
        case 3:
            new_state.left = HIGH;
            break;
        case 4:
            new_state.action = HIGH;
            break;
        case 5:
            new_state.right = HIGH;
            break;
        case 7:
            new_state.down = HIGH;
            break;
    }

#else
    // прочитать все пины
    new_state.up = !bool(digitalRead(BUTTON_UP));
    new_state.down = !bool(digitalRead(BUTTON_DOWN));
    new_state.left = !bool(digitalRead(BUTTON_LEFT));
    new_state.right = !bool(digitalRead(BUTTON_RIGHT));
    new_state.action = !bool(digitalRead(BUTTON_ACTION));
#endif

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
#ifdef DEBUG_SEND
        if (any_active) {
            Serial.println("any active, trying to send a message");
        }
        if (state_changed) {
            Serial.println("state changed, trying to send a message");
        }
#endif
        esp_now_send(PEER_ADDR, (uint8_t *) &new_state, sizeof(new_state));
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


#if defined(ARDUINO_ARCH_ESP8266)
    bool is_stop_left = !(analogRead(STOP_LEFT) > STOP_LEFT_THRESHOLD);
#elif defined(ARDUINO_ARCH_ESP32)
    bool is_stop_left = !digitalRead(STOP_LEFT);
#endif
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
        new_state.up = LOW;
        new_state.down = LOW;
        new_state.left = LOW;
        new_state.right = LOW;
        new_state.action = LOW;
    }

    if ( (!new_state.down && state.down) || (!new_state.up && state.up) ) {
        // кнопка опущена, включить задержку включения этой оси
        updown_blocked_at = millis();
    }

    if ( (!new_state.left && state.left) || (!new_state.right && state.right) ) {
        // кнопка опущена, включить задержку включения этой оси
        leftright_blocked_at = millis();
    }

    if ((millis() - updown_blocked_at) < BLOCK_TIMEOUT) {
#ifdef DEBUG_STOP
        Serial.println("axis reversion block is active (up/down)");
#endif
        new_state.up = LOW;
        new_state.down = LOW;
    }

    if ((millis() - leftright_blocked_at) < BLOCK_TIMEOUT) {
#ifdef DEBUG_STOP
        Serial.println("axis reversion block is active (left/right)");
#endif
        new_state.left = LOW;
        new_state.right = LOW;
    }

    {
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

    if (delay_updown > 0) {
        delay_updown--;
    }

    if (delay_leftright > 0) {
        delay_leftright--;
    }

#endif

#ifdef TOUCH_UI
    // обработка пользовательского интерфейса
    lv_timer_handler();

#ifdef SCREENSAVER
    // уменьшить яркость в простое
    if (brightness == HIGH && (millis() - last_touch_event_at) > SCREENSAVER_TIMEOUT) {
        brightness = LOW;
        digitalWrite(TFT_BL, brightness);
    }
    if (brightness == LOW && (millis() - last_touch_event_at) < SCREENSAVER_TIMEOUT) {
        brightness = HIGH;
        digitalWrite(TFT_BL, brightness);
    }
#endif

#endif /* ifdef TOUCH_UI */

#ifdef REPORT_MESSAGES

#ifdef ROLE_RECV
    if ((millis() - last_report_at) > HEALTH_INTERVAL) {
        static struct_report_message send = {health};
        esp_now_send(PEER_ADDR, (uint8_t *) &send, sizeof(send));
#ifdef DEBUG_SEND
        Serial.println("trying to send health notification msg");
#endif
        // reset counter
        last_report_at = millis();
    }
#else
#ifdef TOUCH_UI
    if ((millis() - last_report_at) > HEALTH_INTERVAL * 2) {
        lv_led_off(led);
    } else {
        lv_led_on(led);
    }
#endif /* ifdef TOUCH_UI */
#endif /* ifdef ROLE_RECV */
#endif /* ifdef REPORT_MESSAGES */
    // задержка до следующей итерации
    delay(LOOP_INTERVAL);
}
