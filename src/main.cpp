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
extern lv_obj_t *ui_ParkingBrake; // Parking brake indicator
extern lv_obj_t *ui_TurnSignal; // Turn signal indicator
extern lv_obj_t *ui_CheckEngine; // Check engine light indicator
extern lv_obj_t *ui_DSC; // DSC indicator

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
#define LVGL_TICK_PERIOD_MS     (2) // 500 Hz for timer
#define LVGL_TASK_MAX_DELAY_MS  (100)
#define LVGL_TASK_MIN_DELAY_MS  (2) // ~500 Hz for active updates
#define LVGL_TASK_IDLE_DELAY_MS (20) // ~50 Hz when idle
#define LVGL_TASK_STACK_SIZE    (6 * 1024)
#define LVGL_TASK_PRIORITY      (5)

// Explicitly define bus type for RGB
#define ESP_PANEL_LCD_BUS_TYPE 3 // 3=RGB in this driver

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
static uint32_t lastDataUpdateTime = 0; // Track last data update for timeout
#define DATA_TIMEOUT_MS 1000 // Timeout to set indicators to disabled

// Track previous states to avoid redundant updates
static uint16_t prev_rpm = 0;
static uint8_t prev_speed = 0;
static bool prev_parkingBrake = false;
static bool prev_turnSignal = false;
static bool prev_checkEngine = false;
static bool prev_dsc = false;

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;
esp_lcd_panel_handle_t lcd_handle = NULL; // For direct DMA calls

/* Debug logging macro */
#define ESPNOW_DEBUG Serial.printf
#define DEBUG_FLUSH 0 // Disable flush logging to reduce overhead

/* Counters for frequency measurement */
static volatile uint32_t espnow_packet_count = 0;
static uint32_t lvgl_update_count = 0;
static uint32_t last_freq_log_time = 0;
const uint32_t freq_log_interval = 1000; // Log frequency every 1000 ms

/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    esp_log_level_set("esp_lcd", ESP_LOG_INFO);
    static uint32_t last_log_time = 0;
    uint32_t start = micros();
    uint32_t pixels = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    esp_err_t ret = esp_lcd_panel_draw_bitmap(lcd_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    uint32_t draw_time = micros() - start;
    uint32_t current_time = millis();
    #if DEBUG_FLUSH
    // Log only if draw time is high or every 1 second
    if (draw_time > 1000 || current_time - last_log_time >= 1000) {
        Serial.printf("Flush: Buffer address: %p, Alignment: %u\n", color_p, (uint32_t)color_p % 32);
        Serial.printf("Draw time: %u us for %dx%d (%u pixels), Return: %s (0x%x)\n", draw_time,
                      area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, pixels,
                      esp_err_to_name(ret), ret);
        last_log_time = current_time;
    }
    #endif
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

void update_indicator_state(lv_obj_t *obj, bool enabled)
{
    if (enabled) {
        // Bright red for enabled state
        lv_obj_set_style_bg_color(obj, lv_color_hex(0xFF0000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_img_opa(obj, LV_OPA_COVER, 0); // For image-based indicators
    } else {
        // Almost transparent red for disabled state
        lv_obj_set_style_bg_color(obj, lv_color_hex(0xFF0000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(obj, LV_OPA_10, LV_PART_MAIN);
        lv_obj_set_style_img_opa(obj, LV_OPA_10, 0); // For image-based indicators
    }
    lv_area_t area;
    lv_obj_get_coords(obj, &area); // Get bounding box
    lv_obj_invalidate_area(obj, &area); // Invalidate only the indicator's area
}

void lvgl_port_task(void *arg)
{
    Serial.println("Starting LVGL task");
    static uint32_t last_fps_time = 0;
    static uint32_t fps_count = 0;
    while (1) {
        lvgl_port_lock(-1);
        uint32_t current_time = millis();
        bool updated = false;

        // Check for data timeout to set indicators to disabled
        if (current_time - lastDataUpdateTime > DATA_TIMEOUT_MS) {
            if (prev_parkingBrake) {
                update_indicator_state(ui_ParkingBrake, false);
                prev_parkingBrake = false;
                updated = true;
            }
            if (prev_turnSignal) {
                update_indicator_state(ui_TurnSignal, false);
                prev_turnSignal = false;
                updated = true;
            }
            if (prev_checkEngine) {
                update_indicator_state(ui_CheckEngine, false);
                prev_checkEngine = false;
                updated = true;
            }
            if (prev_dsc) {
                update_indicator_state(ui_DSC, false);
                prev_dsc = false;
                updated = true;
            }
        }

        if (dataReceived) {
            // Batch all updates
            if (receivedData.rpm != prev_rpm) {
                lv_arc_set_value(ui_rpmslider, receivedData.rpm);
                char rpm_text[16];
                snprintf(rpm_text, sizeof(rpm_text), "%u", receivedData.rpm);
                lv_label_set_text(ui_RPM, rpm_text);
                lv_area_t area;
                lv_obj_get_coords(ui_rpmslider, &area);
                lv_obj_invalidate_area(ui_rpmslider, &area);
                lv_obj_get_coords(ui_RPM, &area);
                lv_obj_invalidate_area(ui_RPM, &area);
                prev_rpm = receivedData.rpm;
                updated = true;
            }

            if (receivedData.vehicleSpeed != prev_speed) {
                char speed_text[16];
                snprintf(speed_text, sizeof(speed_text), "%u", receivedData.vehicleSpeed);
                lv_label_set_text(ui_Speed, speed_text);
                lv_area_t area;
                lv_obj_get_coords(ui_Speed, &area);
                lv_obj_invalidate_area(ui_Speed, &area);
                prev_speed = receivedData.vehicleSpeed;
                updated = true;
            }

            if (receivedData.handbrakeSwitch != prev_parkingBrake) {
                update_indicator_state(ui_ParkingBrake, receivedData.handbrakeSwitch);
                prev_parkingBrake = receivedData.handbrakeSwitch;
                updated = true;
            }

            bool turnSignalState = receivedData.turnSignalIndicator != 0;
            if (turnSignalState != prev_turnSignal) {
                update_indicator_state(ui_TurnSignal, turnSignalState);
                prev_turnSignal = turnSignalState;
                updated = true;
            }

            if (receivedData.checkEngineLight != prev_checkEngine) {
                update_indicator_state(ui_CheckEngine, receivedData.checkEngineLight);
                prev_checkEngine = receivedData.checkEngineLight;
                updated = true;
            }

            if (receivedData.ascLampStatus != prev_dsc) {
                update_indicator_state(ui_DSC, receivedData.ascLampStatus);
                prev_dsc = receivedData.ascLampStatus;
                updated = true;
            }

            lastDataUpdateTime = current_time;
            dataReceived = false;
        }

        // Process LVGL updates and refresh only if something changed
        if (updated) {
            lv_task_handler();
            lv_refr_now(NULL);
            lvgl_update_count++;
            fps_count++;
            if (current_time - last_fps_time >= 1000) {
                Serial.printf("FPS: %u\n", fps_count);
                fps_count = 0;
                last_fps_time = current_time;
            }
        }

        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(dataReceived ? LVGL_TASK_MIN_DELAY_MS : LVGL_TASK_IDLE_DELAY_MS));
    }
}

/* ESP-NOW callback function with throttled debugging */
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    static uint32_t last_log_time = 0;
    const uint32_t log_interval = 1000; // Log every 1000 ms

    if (len != sizeof(CanData)) {
        ESPNOW_DEBUG("ESP-NOW: Error: Invalid data length, expected %d, got %d\n", sizeof(CanData), len);
        return;
    }

    uint32_t current_time = millis();
    // Only process if enough time has passed (to enforce ~20 Hz)
    if (current_time - lastPacketTime >= 40) { // ~25 Hz to allow jitter
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
    WiFi.setSleep(false); // Disable WiFi power saving for DMA performance
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

    /* Initialize LVGL buffers in DMA-capable memory with 75 lines */
    uint32_t buffer_size = ESP_PANEL_LCD_H_RES * 75 * sizeof(lv_color_t);
    lv_color_t *buf1 = (lv_color_t *)heap_caps_aligned_alloc(32, buffer_size, MALLOC_CAP_DMA);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_aligned_alloc(32, buffer_size, MALLOC_CAP_DMA);

    static lv_disp_draw_buf_t draw_buf;
    static lv_disp_drv_t disp_drv;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, ESP_PANEL_LCD_H_RES * 75);
    lv_disp_drv_init(&disp_drv);

    if (buf1 && buf2) {
        Serial.printf("Double buffering enabled in DMA-capable memory, buffer size: %u bytes each (800x75)\n",
                     buffer_size);
        Serial.printf("Free SRAM after setup: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        Serial.printf("Free PSRAM after setup: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        Serial.printf("Buffer 1 address: %p, Alignment: %u\n", buf1, (uint32_t)buf1 % 32);
        Serial.printf("Buffer 2 address: %p, Alignment: %u\n", buf2, (uint32_t)buf2 % 32);
    } else {
        Serial.println("ERROR: Failed to allocate double buffers in DMA-capable memory");
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
    lcd_handle = panel->getLcd()->getHandle(); // Get panel handle for direct DMA calls

    /* Log bus type and verify */
    Serial.printf("LCD Bus Type: %d (3=RGB)\n", ESP_PANEL_LCD_BUS_TYPE);
    #if ESP_PANEL_LCD_BUS_TYPE == 3 // RGB
        // Attempt to set pixel clock to 15 MHz (adjust based on panel datasheet)
        // Note: ESP_Panel_Library may not expose this directly; check board settings
        // panel->configLcdBusFrequency(15000000); // Uncomment if supported
        Serial.println("RGB mode: Pixel clock configuration not directly accessible. Check board settings (e.g., board.h).");
        Serial.println("RGB burst mode: Typically not applicable (continuous transfer)");
        Serial.println("Burst mode enabled or assumed for optimal performance");
    #elif ESP_PANEL_LCD_BUS_TYPE == 1 // SPI
        panel->configLcdBusFrequency(40000000); // Set to 40 MHz for SPI
        panel->init(); // Re-init to apply frequency
        Serial.printf("SPI Frequency: %u Hz\n", panel->getLcdBusFrequency());
        Serial.println("SPI burst mode: Assuming enabled (verify with panel library)");
    #elif ESP_PANEL_LCD_BUS_TYPE == 2 // I2C
        Serial.printf("I2C Frequency: %u Hz\n", panel->getLcdBusFrequency());
        Serial.println("Burst mode: Not applicable for I2C");
    #else
        Serial.println("ERROR: Unsupported LCD Bus Type. Expected 3 (RGB), 1 (SPI), or 2 (I2C).");
    #endif

    #if ESP_PANEL_LCD_BUS_TYPE != 3
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
    // Verify ui_rpmslider, ui_RPM, ui_Speed, and indicators
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
    if (ui_ParkingBrake == NULL) {
        Serial.println("ERROR: ui_ParkingBrake is NULL, check ui_init()");
        while (1);
    }
    if (ui_TurnSignal == NULL) {
        Serial.println("ERROR: ui_TurnSignal is NULL, check ui_init()");
        while (1);
    }
    if (ui_CheckEngine == NULL) {
        Serial.println("ERROR: ui_CheckEngine is NULL, check ui_init()");
        while (1);
    }
    if (ui_DSC == NULL) {
        Serial.println("ERROR: ui_DSC is NULL, check ui_init()");
        while (1);
    }
    Serial.println("ui_rpmslider, ui_RPM, ui_Speed, ui_ParkingBrake, ui_TurnSignal, ui_CheckEngine, ui_DSC initialized successfully");
    // Ensure widgets are not hidden
    lv_obj_clear_flag(ui_rpmslider, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_RPM, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Speed, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ParkingBrake, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_TurnSignal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_CheckEngine, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_DSC, LV_OBJ_FLAG_HIDDEN);
    // Set arc range to 0-11000
    lv_arc_set_range(ui_rpmslider, 0, 11000);
    lv_arc_set_value(ui_rpmslider, 0);
    lv_label_set_text(ui_RPM, "0");
    lv_label_set_text(ui_Speed, "0");
    // Initialize indicators to disabled state
    lv_obj_set_style_bg_color(ui_ParkingBrake, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_ParkingBrake, LV_OPA_10, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_ParkingBrake, LV_OPA_10, 0);
    lv_obj_set_style_bg_color(ui_TurnSignal, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_TurnSignal, LV_OPA_10, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_TurnSignal, LV_OPA_10, 0);
    lv_obj_set_style_bg_color(ui_CheckEngine, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_CheckEngine, LV_OPA_10, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_CheckEngine, LV_OPA_10, 0);
    lv_obj_set_style_bg_color(ui_DSC, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_DSC, LV_OPA_10, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_DSC, LV_OPA_10, 0);
    lv_obj_invalidate(ui_ParkingBrake);
    lv_obj_invalidate(ui_TurnSignal);
    lv_obj_invalidate(ui_CheckEngine);
    lv_obj_invalidate(ui_DSC);
    lv_obj_invalidate(ui_rpmslider);
    lv_obj_invalidate(ui_RPM);
    lv_obj_invalidate(ui_Speed);
    lv_refr_now(NULL);
    Serial.println("Initial RPM set to 0, Speed set to 0 km/h, indicators set to disabled");
    lvgl_port_unlock();

    lastDataUpdateTime = millis();
    Serial.println("Setup done");
}

void loop()
{
    uint32_t current_time = millis();
    if (current_time - last_freq_log_time >= freq_log_interval) {
        float espnow_hz = (float)espnow_packet_count * 1000.0f / freq_log_interval;
        float lvgl_hz = (float)lvgl_update_count * 1000.0f / freq_log_interval;
        ESPNOW_DEBUG("FREQ: ESP-NOW Packet Hz: %.2f, LVGL Update Hz: %.2f\n", espnow_hz, lvgl_hz);
        Serial.printf("Free stack (LVGL task): %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
        espnow_packet_count = 0;
        lvgl_update_count = 0;
        last_freq_log_time = current_time;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}