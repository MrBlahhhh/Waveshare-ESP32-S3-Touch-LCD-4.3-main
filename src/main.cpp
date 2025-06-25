#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include <esp_now.h>
#include <WiFi.h>

// Declare UI objects from ui_screen1.c
extern lv_obj_t *ui_rpmslider;

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
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 40)

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

/* Debug logging macro */
#define ESPNOW_DEBUG Serial.printf

/* FPS label (fallback) */
static lv_obj_t *fps_label = NULL;

/* Manual FPS counter variables */
static volatile uint32_t frame_count = 0;
static uint32_t last_fps = 0;

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
        frame_count++; // Increment frame count for manual FPS
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

/* ESP-NOW callback function with debugging */
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    ESPNOW_DEBUG("ESP-NOW: Data received, length: %d bytes\n", len);
    if (len != sizeof(CanData)) {
        ESPNOW_DEBUG("ESP-NOW: Error: Invalid data length, expected %d, got %d\n", sizeof(CanData), len);
        return;
    }

    memcpy(&receivedData, incomingData, sizeof(CanData));
    ESPNOW_DEBUG("ESP-NOW: Received RPM: %u, Coolant Temp: %u, Vehicle Speed: %u\n",
                 receivedData.rpm, receivedData.coolantTemp, receivedData.vehicleSpeed);
    ESPNOW_DEBUG("ESP-NOW: Source MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    dataReceived = true;
}

/* FPS update callback for fallback label */
static void fps_update_cb(lv_timer_t *timer)
{
    if (fps_label) {
        // Calculate FPS based on frame_count over 1 second
        uint32_t fps = frame_count;
        frame_count = 0; // Reset for next second
        char buf[16];
        snprintf(buf, sizeof(buf), "FPS: %lu", fps);
        lv_label_set_text(fps_label, buf);
        if (fps != last_fps) {
            ESPNOW_DEBUG("Manual FPS: %lu\n", fps);
            last_fps = fps;
        }
    }
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
    ESPNOW_DEBUG("ESP-NOW: Initializing WiFi in STA mode\n");
    WiFi.mode(WIFI_STA);
    ESPNOW_DEBUG("ESP-NOW: WiFi MAC Address: %s\n", WiFi.macAddress().c_str());

    /* Initialize ESP-NOW */
    ESPNOW_DEBUG("ESP-NOW: Initializing ESP-NOW\n");
    esp_err_t init_result = esp_now_init();
    if (init_result != ESP_OK) {
        ESPNOW_DEBUG("ESP-NOW: Error initializing ESP-NOW: %s (0x%x)\n", 
                     esp_err_to_name(init_result), init_result);
        return;
    }
    ESPNOW_DEBUG("ESP-NOW: Initialization successful\n");

    /* Register ESP-NOW receive callback */
    ESPNOW_DEBUG("ESP-NOW: Registering receive callback\n");
    esp_err_t cb_result = esp_now_register_recv_cb(OnDataRecv);
    if (cb_result != ESP_OK) {
        ESPNOW_DEBUG("ESP-NOW: Error registering callback: %s (0x%x)\n", 
                     esp_err_to_name(cb_result), cb_result);
        return;
    }
    ESPNOW_DEBUG("ESP-NOW: Receive callback registered\n");

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize system monitor for default FPS display */
    _lv_sysmon_builtin_init();
    Serial.println("LVGL system monitor initialized");

    /* Initialize LVGL buffers */
    uint8_t *buf1 = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    uint8_t *buf2 = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    
    lv_display_t *disp = lv_display_create(ESP_PANEL_LCD_H_RES, ESP_PANEL_LCD_V_RES);
    if (buf1 && buf2) {
        Serial.println("Double buffering enabled");
        lv_display_set_buffers(disp, buf1, buf2, LVGL_BUF_SIZE, LV_DISP_RENDER_MODE_PARTIAL);
    } else {
        Serial.println("Double buffering failed, falling back to single buffering");
        if (buf1) heap_caps_free(buf1);
        if (buf2) heap_caps_free(buf2);
        buf1 = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
        if (!buf1) {
            Serial.println("Failed to allocate even single LVGL buffer");
            while (1);
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
    // Comment out ui_init() temporarily to test if default FPS counter appears
    ui_init();
    // Verify ui_rpmslider is initialized
    if (ui_rpmslider == NULL) {
        Serial.println("ERROR: ui_rpmslider is NULL, check ui_init()");
        while (1); // Halt to indicate critical error
    } else {
        Serial.println("ui_rpmslider initialized successfully");
        // Set arc range (adjust as needed for your RPM range)
        lv_arc_set_range(ui_rpmslider, 0, 8000);
        lv_arc_set_value(ui_rpmslider, 0); // Initialize to 0
    }

    // Create fallback FPS label
    fps_label = lv_label_create(lv_screen_active());
    lv_label_set_text(fps_label, "FPS: --");
    lv_obj_set_pos(fps_label, 10, 10); // Top-left corner
    lv_obj_set_style_text_color(fps_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(fps_label, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(fps_label, LV_OPA_50, 0);
    // Create timer to update FPS
    lv_timer_create(fps_update_cb, 1000, NULL); // Update every 1 second
    Serial.println("Fallback FPS label created");

    lvgl_port_unlock();

    Serial.println("Setup done");
}

void loop()
{
    if (dataReceived) {
        lvgl_port_lock(-1);
        // Debug the RPM value being set
        ESPNOW_DEBUG("LVGL: Setting ui_rpmslider to RPM: %u\n", receivedData.rpm);
        lv_arc_set_value(ui_rpmslider, receivedData.rpm);
        // Force display refresh
        lv_refr_now(NULL);
        lvgl_port_unlock();
        ESPNOW_DEBUG("ESP-NOW: Updated RPM slider to %u\n", receivedData.rpm);
        dataReceived = false;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Reduced delay for faster updates
}