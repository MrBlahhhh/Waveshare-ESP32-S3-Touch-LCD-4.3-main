#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include <esp_now.h>
#include <WiFi.h>

// Declare UI objects from ui_screen1.c
extern lv_obj_t *ui_rpmslider;
extern lv_obj_t *ui_RPM; // Label for RPM display
extern lv_obj_t *ui_Speed; // Label for speed display

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
#define LVGL_TASK_MIN_DELAY_MS  (5) // ~200 Hz to catch 20 Hz packets
#define LVGL_TASK_STACK_SIZE    (6 * 1024)
#define LVGL_TASK_PRIORITY      (5)

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
volatile uint32_t lastPacketTime = 0; // Track packet timing

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;

/* Debug logging macro */
#define ESPNOW_DEBUG Serial.printf

/* Counters for frequency measurement */
static volatile uint32_t espnow_packet_count = 0;
static uint32_t lvgl_update_count = 0;
static uint32_t last_freq_log_time = 0;
const uint32_t freq_log_interval = 1000; // Log frequency every 1000 ms

/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint16_t *)color_p);
    lv_disp_flush_ready(disp_drv);
}

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
        if (dataReceived) {
            // Update RPM arc and label without smoothing
            uint16_t display_rpm = receivedData.rpm;
            lv_arc_set_value(ui_rpmslider, display_rpm);
            char rpm_text[16];
            snprintf(rpm_text, sizeof(rpm_text), "%u", display_rpm);
            lv_label_set_text(ui_RPM, rpm_text);

            // Update speed label without smoothing
            uint8_t display_speed = receivedData.vehicleSpeed;
            char speed_text[16];
            snprintf(speed_text, sizeof(speed_text), "%u km/h", display_speed);
            lv_label_set_text(ui_Speed, speed_text);

            lv_obj_invalidate(ui_rpmslider); // Force redraw
            lv_obj_invalidate(ui_RPM);
            lv_obj_invalidate(ui_Speed);

            lv_task_handler(); // Process LVGL updates
            lvgl_update_count++; // Increment counter
            dataReceived = false; // Clear flag after processing
        }
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz to catch 20 Hz packets
    }
}

/* ESP-NOW callback function with throttled debugging */
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    static uint32_t last_log_time = 0;
    const uint32_t log_interval = 1000; // Log every 1000 ms

    if (len != sizeof(CanData)) {
        ESPNOW_DEBUG("ESP-NOW: Error: Invalid data length, expected %d, got %d\n", sizeof(CanData), len);
        return;
    }

    uint32_t current_time = millis();
    // Only process if enough time has passed (to enforce ~20 Hz)
    if (current_time - lastPacketTime >= 45) { // ~22 Hz to allow slight jitter
        memcpy(&receivedData, incomingData, sizeof(CanData));
        espnow_packet_count++; // Increment packet counter
        lastPacketTime = current_time;
        dataReceived = true;

        if (current_time - last_log_time >= log_interval) {
            ESPNOW_DEBUG("ESP-NOW: Received RPM: %u, Coolant Temp: %u, Vehicle Speed: %u\n",
                         receivedData.rpm, receivedData.coolantTemp, receivedData.vehicleSpeed);
            ESPNOW_DEBUG("ESP-NOW: Source MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            last_log_time = current_time;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(3000);       

    String LVGL_Arduino = "Hello LVGL! ";
    LVGL_Arduino += String('V') + LVGL_VERSION_MAJOR + "." + LVGL_VERSION_MINOR + "." + LVGL_VERSION_PATCH;
    Serial.println(LVGL_Arduino);
    Serial.println("I am ESP32_Display_Panel");

    /* Log free memory before allocations */
    Serial.printf("Free SRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.printf("Free PSRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

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

    /* Initialize LVGL buffers in SRAM with 25 lines */
    uint32_t buffer_size = ESP_PANEL_LCD_H_RES * 25 * sizeof(lv_color_t);
    lv_color_t *buf1 = (lv_color_t *)heap_caps_calloc(1, buffer_size, MALLOC_CAP_INTERNAL);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_calloc(1, buffer_size, MALLOC_CAP_INTERNAL);
    
    static lv_disp_draw_buf_t draw_buf;
    static lv_disp_drv_t disp_drv;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, ESP_PANEL_LCD_H_RES * 25);
    lv_disp_drv_init(&disp_drv);
    
    if (buf1 && buf2) {
        Serial.printf("Double buffering enabled in SRAM, buffer size: %u bytes each (800x25)\n", 
                     buffer_size);
        Serial.printf("Free SRAM after setup: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    } else {
        Serial.println("ERROR: Failed to allocate double buffers in SRAM");
        if (buf1) heap_caps_free(buf1);
        if (buf2) heap_caps_free(buf2);
        Serial.printf("Free SRAM after failure: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        while (1); // Halt on failure
    }

    /* Initialize the display device */
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    /* Initialize bus and device of panel */
    panel->init();
#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    Serial.println("Initialize IO expander");
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);

    expander->digitalWrite(USB_SEL, LOW);
    panel->addIOExpander(expander);

    /* Reset LCD to ensure proper initialization */
    expander->digitalWrite(LCD_RST, LOW);
    delay(100);
    expander->digitalWrite(LCD_RST, HIGH);
    delay(100);

    /* Start panel */
    panel->begin();

    /* Create a task to run the LVGL task periodically */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    lvgl_port_lock(-1);
    ui_init();
    // Verify ui_rpmslider, ui_RPM, and ui_Speed
    if (ui_rpmslider == NULL) {
        Serial.println("ERROR: ui_rpmslider is NULL, check ui_init()");
        while (1);
    }
    if (ui_RPM == NULL) {
        Serial.println("ERROR: ui_RPM is NULL, check ui_init()");
        while (1);
    }
    if (ui_Speed == NULL) {
        Serial.println("ERROR: ui_Speed is NULL, check ui_init()");
        while (1);
    }
    Serial.println("ui_rpmslider, ui_RPM, and ui_Speed initialized successfully");
    // Ensure widgets are not hidden
    lv_obj_clear_flag(ui_rpmslider, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_RPM, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Speed, LV_OBJ_FLAG_HIDDEN);
    // Set arc range to 0-11000
    lv_arc_set_range(ui_rpmslider, 0, 11000);
    lv_arc_set_value(ui_rpmslider, 0);
    lv_label_set_text(ui_RPM, "0");
    lv_label_set_text(ui_Speed, "0 km/h");
    Serial.println("Initial RPM set to 0, Speed set to 0 km/h");
    lv_obj_invalidate(ui_rpmslider); // Force initial redraw
    lv_obj_invalidate(ui_RPM);
    lv_obj_invalidate(ui_Speed);
    lv_refr_now(NULL); // Force initial screen refresh
    Serial.println("Initial screen refresh completed");
    lvgl_port_unlock();

    Serial.println("Setup done");
}

void loop()
{
    // Log frequency metrics
    uint32_t current_time = millis();
    if (current_time - last_freq_log_time >= freq_log_interval) {
        float espnow_hz = (float)espnow_packet_count * 1000.0f / freq_log_interval;
        float lvgl_hz = (float)lvgl_update_count * 1000.0f / freq_log_interval;
        ESPNOW_DEBUG("FREQ: ESP-NOW Packet Hz: %.2f, LVGL Update Hz: %.2f\n", espnow_hz, lvgl_hz);
        espnow_packet_count = 0; // Reset counters
        lvgl_update_count = 0;
        last_freq_log_time = current_time;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms for frequency logging
}