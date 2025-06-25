#include <Arduino.h>

/**
 * The example demonstrates how to port LVGL 9.1.0.
 *
 * ## How to Use
 *
 * To use this example, please firstly install `ESP32_Display_Panel` (including its dependent libraries) and
 * `lvgl` (v9.1.0) libraries, then follow the steps to configure them:
 *
 * 1. [Configure ESP32_Display_Panel](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-esp32_display_panel)
 * 2. [Configure LVGL](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-lvgl)
 * 3. [Configure Board](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-board)
 *
 * ## Example Output
 *
 * ```bash
 * ...
 * Hello LVGL! V9.1.0
 * I am ESP32_Display_Panel
 * Starting LVGL task
 * Setup done
 * Loop
 * Loop
 * Loop
 * Loop
 * ...
 * ```
 */

#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include <esp_now.h>
#include <WiFi.h>

// Declare UI objects from ui_screen1.c
extern lv_obj_t *ui_rpmslider;
extern lv_obj_t *ui_parkingbrake;

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (100)
#define LVGL_TASK_MIN_DELAY_MS  (5)
#define LVGL_TASK_STACK_SIZE    (6 * 1024)
#define LVGL_TASK_PRIORITY      (3)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 100) // Reduced to 100 lines per buffer

/* ESP-NOW data structure */
typedef struct {
  uint16_t rpm;
  uint8_t coolantTemp;
  uint8_t angleFgrPedal;
  uint8_t driverDemand;
  uint8_t checkEngineLight : 1;
  uint8_t engineWarningLight : 1;
  uint8_t boostFailureLight : 1;
  uint8_t overheating : 1;
  uint8_t manifoldPressure;
  uint8_t fuelTankLevel;
  uint8_t switchFillingStatus : 1;
  uint8_t handbrakeSwitch : 1;
  uint8_t turnSignalIndicator : 2;
  uint8_t odbFault : 1;
  uint8_t manualGearSelected : 4;
  uint8_t requestAsc : 1;
  uint8_t requestMsr : 1;
  uint8_t ascLampStatus : 1;
  uint8_t vehicleSpeed : 5;
} CanData;

CanData receivedData;
volatile bool dataReceived = false;

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;

/* Display flushing */
void lvgl_port_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (lv_color_t *)color_p);
    lv_display_flush_ready(disp);
}

#if ESP_PANEL_USE_LCD_TOUCH
/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    panel->getLcdTouch()->readData();
    bool touched = panel->getLcdTouch()->getTouchState();
    if (!touched) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = point.x;
        data->point.y = point.y;
    }
}
#endif

void lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

void lvgl_port_task(void *arg)
{
    Serial.println("Starting LVGL task");
    while (1) {
        lvgl_port_lock(-1);
        lv_timer_handler();
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

/* ESP-NOW callback function */
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    dataReceived = true;
}

void setup()
{
    Serial.begin(115200);
    delay(3000);       

    String LVGL_Arduino = "Hello LVGL! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
    Serial.println(LVGL_Arduino);
    Serial.println("I am ESP32_Display_Panel");

    /* Log free memory before allocations */
    Serial.printf("Free SRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    #ifdef CONFIG_SPIRAM
    Serial.printf("Free PSRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    #endif

    /* Initialize WiFi for ESP-NOW */
    WiFi.mode(WIFI_STA);
    Serial.println("WiFi MAC Address: ");
    Serial.println(WiFi.macAddress());

    /* Initialize ESP-NOW */
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    /* Register ESP-NOW receive callback */
    esp_now_register_recv_cb(OnDataRecv);

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize LVGL buffers */
    uint8_t *buf1 = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    uint8_t *buf2 = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    
    lv_display_t *disp = lv_display_create(ESP_PANEL_LCD_H_RES, ESP_PANEL_LCD_V_RES);
    if (buf1 && buf2) {
        Serial.println("Double buffering enabled");
        lv_display_set_buffers(disp, buf1, buf2, LVGL_BUF_SIZE, LV_DISP_RENDER_MODE_PARTIAL);
    } else {
        Serial.println("Double buffering failed, falling back to single buffering");
        if (buf1) heap_caps_free(buf1); // Free buf1 if allocated
        if (buf2) heap_caps_free(buf2); // Free buf2 if allocated
        buf1 = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
        if (!buf1) {
            Serial.println("Failed to allocate even single LVGL buffer");
            while (1); // Halt on failure
        }
        lv_display_set_buffers(disp, buf1, NULL, LVGL_BUF_SIZE, LV_DISP_RENDER_MODE_PARTIAL);
    }

    /* Initialize the display device */
    lv_display_set_resolution(disp, ESP_PANEL_LCD_H_RES, ESP_PANEL_LCD_V_RES);
    lv_display_set_flush_cb(disp, lvgl_port_disp_flush);

#if ESP_PANEL_USE_LCD_TOUCH
    /* Initialize the input device */
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_port_tp_read);
#endif

    /* Initialize bus and device of panel */
    panel->init();
#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, disp);
#endif

    Serial.println("Initialize IO expander");
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);

    expander->digitalWrite(USB_SEL, LOW);
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create a task to run the LVGL task periodically */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    lvgl_port_lock(-1);
    ui_init();
    lvgl_port_unlock();

    Serial.println("Setup done");
}

void loop()
{
    if (dataReceived) {
        lvgl_port_lock(-1);
        lv_arc_set_value(ui_rpmslider, receivedData.rpm);
        lv_obj_set_style_img_recolor_opa(ui_parkingbrake, receivedData.handbrakeSwitch ? 0 : 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lvgl_port_unlock();
        dataReceived = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}