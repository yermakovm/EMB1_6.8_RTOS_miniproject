/*
 * Power-aware data logger on FreeRTOS (ESP32-S3, ESP-IDF).
 *
 * 5 tasks: Sensor / UART / Display / Manager / Flash Writer.
 * Queues: sensor_queue, command_queue, display_queue, flash_queue, flash_result_queue.
 * Mutex: one I2C bus mutex (BME280 + DS1307 + SSD1306 share the bus).
 * Light Sleep: entered automatically by the FreeRTOS IDLE task (tickless idle),
 * see CONFIG_PM_ENABLE / CONFIG_FREERTOS_USE_TICKLESS_IDLE in sdkconfig.defaults.
 *
 * Architecture and diagrams: README.md + doc/uml/.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"

#include "font8x8.h"

static const char *TAG = "logger";

/* ------------------------------ wiring ------------------------------ */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     5
#define I2C_SCL_PIN     4
#define BME280_ADDR     0x76
#define DS1307_ADDR     0x68   // "DS1306" in the task statement
#define SSD1306_ADDR    0x3C

/* --------------------------- logging policy ------------------------- */
#define WAKE_PERIOD_MS        1000   // one measurement + display cycle
#define LOG_INTERVAL_MINUTES  1      // log to flash once every N minutes
#define FLASH_SECTOR_SIZE     4096   // RAM log buffer size = 1 flash sector
#define LOG_PARTITION_LABEL   "logdata"
#define DUMP_DEFAULT_RECORDS  20

/* ------------------------------ devices ----------------------------- */
#define BME280_REG_ID         0xD0
#define BME280_ID             0x60
#define BME280_REG_SOFTRESET  0xE0
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_STATUS     0xF3
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_PRESS      0xF7   // press[3], hum[2], temp[3]

/* ------------------------------ display ----------------------------- */
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_COLS   (OLED_WIDTH / 8)   // 16 characters per line
#define OLED_ROWS   (OLED_HEIGHT / 8)  // 8 text lines

/* ---------------------------- data types ---------------------------- */
typedef struct {
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    int16_t  temperature_c_x100;
    uint16_t humidity_pct_x100;
    uint16_t pressure_mmhg;
} log_record_t;

#define RECORDS_PER_SECTOR (FLASH_SECTOR_SIZE / sizeof(log_record_t))
_Static_assert(RECORDS_PER_SECTOR * sizeof(log_record_t) <= FLASH_SECTOR_SIZE,
               "one flash sector must hold a whole number of records");

typedef struct {
    uint8_t second, minute, hour, day, month, year; // decoded BCD, year = 2000 + value
} rtc_time_t;

typedef struct {
    log_record_t record;
    uint16_t buffer_used;
    bool logging_enabled;
    bool partition_full;
    bool sensor_ok;
} display_snapshot_t;

typedef enum {
    CMD_HELP, CMD_STATUS, CMD_LOG_ON, CMD_LOG_OFF, CMD_FLUSH, CMD_ERASE, CMD_DUMP, CMD_SET_TIME
} command_type_t;

typedef struct {
    int16_t year;
    uint8_t month, day, hour, minute, second;
} rtc_time_w_t;

typedef struct {
    command_type_t type;
    int32_t argument;
    rtc_time_w_t rtc;
} uart_command_t;

typedef struct {
    const uint8_t *data;
    size_t len;
} flash_req_t;

typedef enum { FLASH_OK, FLASH_FULL, FLASH_ERR } flash_result_t;

/* --------------------------- RTOS objects --------------------------- */
static SemaphoreHandle_t s_i2c_mutex;
static QueueHandle_t s_sensor_q;      // Sensor Task  -> Manager Task
static QueueHandle_t s_cmd_q;         // UART Task    -> Manager Task
static QueueHandle_t s_display_q;     // Manager Task -> Display Task
static QueueHandle_t s_flash_q;       // Manager Task -> Flash Writer Task
static QueueHandle_t s_flash_result_q;// Flash Writer -> Manager Task

#define I2C_LOCK()   xSemaphoreTake(s_i2c_mutex, portMAX_DELAY)
#define I2C_UNLOCK() xSemaphoreGive(s_i2c_mutex)

/* --------------------------- peripherals ---------------------------- */
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_bme_dev;
static i2c_master_dev_handle_t s_rtc_dev;
static esp_lcd_panel_handle_t  s_panel;
static uint8_t s_oled_fb[OLED_WIDTH * OLED_HEIGHT / 8];

/* ---------------------------- flash state --------------------------- */
static const esp_partition_t *s_log_part;
static volatile size_t   s_flash_offset;          // next sector to write
static volatile uint16_t s_last_sector_records;   // records in the last written sector

/* ---------------------------- log buffer ---------------------------- */
static uint8_t s_log_buf[FLASH_SECTOR_SIZE];      // the one RAM buffer = 1 flash sector
static size_t  s_log_buf_len;

/* ============================== I2C ================================= */
static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(dev, buf, sizeof(buf), 1000);
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, out, len, 1000);
}

/* ============================= BME280 =============================== */
typedef struct {
    uint16_t t1; int16_t t2, t3;
    uint16_t p1; int16_t p2, p3, p4, p5, p6, p7, p8, p9;
    uint8_t h1; int16_t h2; uint8_t h3; int16_t h4, h5; int8_t h6;
} bme280_calib_t;

static bme280_calib_t s_bme;
static int32_t s_t_fine;

// BME280 calibration words are stored little-endian
static uint16_t le16(const uint8_t *p) { return (uint16_t)((p[1] << 8) | p[0]); }

static esp_err_t bme280_init(void)
{
    I2C_LOCK();
    esp_err_t err = i2c_write_reg(s_bme_dev, BME280_REG_SOFTRESET, 0xB6);
    I2C_UNLOCK();
    vTaskDelay(pdMS_TO_TICKS(10));

    // After a soft reset the chip reloads its trim data from NVM; reading 0x88 too early
    // returns garbage, which then produces a nonsense temperature.
    for (int i = 0; err == ESP_OK && i < 50; i++) {
        uint8_t status = 0;
        I2C_LOCK();
        err = i2c_read_reg(s_bme_dev, BME280_REG_STATUS, &status, 1);
        I2C_UNLOCK();
        if (err != ESP_OK || (status & 0x01) == 0) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uint8_t id = 0, cal[24], h1 = 0, h[7];
    I2C_LOCK();
    if (err == ESP_OK) err = i2c_read_reg(s_bme_dev, BME280_REG_ID, &id, 1);
    if (err == ESP_OK) err = i2c_read_reg(s_bme_dev, 0x88, cal, sizeof(cal));
    if (err == ESP_OK) err = i2c_read_reg(s_bme_dev, 0xA1, &h1, 1);
    if (err == ESP_OK) err = i2c_read_reg(s_bme_dev, 0xE1, h, sizeof(h));
    // oversampling x1, normal mode, 500 ms standby (DS 5.4.3: CTRL_MEAS is written last)
    if (err == ESP_OK) err = i2c_write_reg(s_bme_dev, BME280_REG_CTRL_HUM, 0x01);
    if (err == ESP_OK) err = i2c_write_reg(s_bme_dev, BME280_REG_CONFIG, 0x80);
    if (err == ESP_OK) err = i2c_write_reg(s_bme_dev, BME280_REG_CTRL_MEAS, 0x27);
    I2C_UNLOCK();

    if (err != ESP_OK) return err;
    if (id != BME280_ID) {
        ESP_LOGW(TAG, "unexpected chip id 0x%02X at address 0x%02X (expected 0x60; a BME280 with SDO high lives at 0x77)",
                 id, BME280_ADDR);
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_bme.t1 = le16(&cal[0]);
    s_bme.t2 = (int16_t)le16(&cal[2]);
    s_bme.t3 = (int16_t)le16(&cal[4]);
    for (int i = 0; i < 9; i++) {
        uint16_t v = le16(&cal[6 + 2 * i]);
        switch (i) {
            case 0: s_bme.p1 = v; break;
            case 1: s_bme.p2 = (int16_t)v; break;
            case 2: s_bme.p3 = (int16_t)v; break;
            case 3: s_bme.p4 = (int16_t)v; break;
            case 4: s_bme.p5 = (int16_t)v; break;
            case 5: s_bme.p6 = (int16_t)v; break;
            case 6: s_bme.p7 = (int16_t)v; break;
            case 7: s_bme.p8 = (int16_t)v; break;
            default: s_bme.p9 = (int16_t)v; break;
        }
    }
    s_bme.h1 = h1;
    s_bme.h2 = (int16_t)le16(&h[0]);
    s_bme.h3 = h[2];
    s_bme.h4 = (int16_t)(((int8_t)h[3] << 4) | (h[4] & 0x0F));
    s_bme.h5 = (int16_t)(((int8_t)h[5] << 4) | (h[4] >> 4));
    s_bme.h6 = (int8_t)h[6];
    return ESP_OK;
}

// Bosch compensation formulas (BME280 datasheet 4.2.3)
static esp_err_t bme280_read(int16_t *temp_c_x100, uint16_t *hum_pct_x100, uint16_t *press_mmhg)
{
    uint8_t d[8];
    I2C_LOCK();
    esp_err_t err = i2c_read_reg(s_bme_dev, BME280_REG_PRESS, d, sizeof(d));
    I2C_UNLOCK();
    if (err != ESP_OK) return err;

    const int32_t adc_p = (int32_t)((d[0] << 12) | (d[1] << 4) | (d[2] >> 4));
    const int32_t adc_h = (int32_t)((d[3] << 8) | d[4]);
    const int32_t adc_t = (int32_t)((d[5] << 12) | (d[6] << 4) | (d[7] >> 4));

    if (adc_t == 0 || adc_t == 0x80000) {
        // No conversion happened: the sensor is still in sleep mode or the read is stale.
        ESP_LOGW(TAG, "BME280 returns no data: adc_t=%d adc_p=%d adc_h=%d (cal T1=%u T2=%d T3=%d)",
                 (int)adc_t, (int)adc_p, (int)adc_h, s_bme.t1, (int)s_bme.t2, (int)s_bme.t3);
        return ESP_ERR_INVALID_RESPONSE;
    }

    int32_t v1 = (adc_t / 8 - (int32_t)s_bme.t1 * 2) * s_bme.t2 / 2048;
    int32_t v2 = (adc_t / 16 - (int32_t)s_bme.t1);
    v2 = (v2 * v2 / 4096) * s_bme.t3 / 16384;
    s_t_fine = v1 + v2;
    *temp_c_x100 = (int16_t)((s_t_fine * 5 + 128) / 256);   // 23.45 C -> 2345

    int64_t p1 = (int64_t)s_t_fine - 128000;
    int64_t p2 = p1 * p1 * s_bme.p6 + (p1 * s_bme.p5) * 131072 + (int64_t)s_bme.p4 * 34359738368LL;
    p1 = (p1 * p1 * s_bme.p3) / 256 + (p1 * s_bme.p2) * 4096;
    p1 = (140737488355328LL + p1) * s_bme.p1 / 8589934592LL;
    if (p1 == 0) {
        *press_mmhg = 0;
    } else {
        int64_t p4 = 1048576 - adc_p;
        p4 = ((p4 * 2147483648LL - p2) * 3125) / p1;
        p1 = (int64_t)s_bme.p9 * (p4 / 8192) * (p4 / 8192) / 33554432LL;
        p2 = (int64_t)s_bme.p8 * p4 / 524288;
        p4 = (p4 + p1 + p2) / 256 + (int64_t)s_bme.p7 * 16;
        const int64_t pa = p4 / 256;                          // Pascal
        *press_mmhg = (uint16_t)(pa * 760 / 101325);         // mmHg
    }

    int32_t h1v = s_t_fine - 76800;
    int32_t h2v = adc_h * 16384;
    int32_t h3v = s_bme.h4 * 1048576;
    int32_t h4v = s_bme.h5 * h1v;
    int32_t h5v = (((h2v - h3v) - h4v) + 16384) / 32768;
    h2v = h1v * s_bme.h6 / 1024;
    h3v = h1v * s_bme.h3 / 2048;
    h4v = (h2v * (h3v + 32768)) / 1024 + 2097152;
    h2v = (h4v * s_bme.h2 + 8192) / 16384;
    h3v = h5v * h2v;
    h4v = (h3v / 32768) * (h3v / 32768) / 128;
    h5v = h3v - h4v * s_bme.h1 / 16;
    if (h5v < 0) h5v = 0;
    if (h5v > 419430400) h5v = 419430400;
    *hum_pct_x100 = (uint16_t)((h5v / 4096) * 100 / 1024);   // 56.78 % -> 5678

    // BME280 spec range is -40..85 C; outside that it is a bad read, not a cold room.
    if (*temp_c_x100 < -4000 || *temp_c_x100 > 8500) {
        const int shown = (int)*temp_c_x100;
        ESP_LOGW(TAG, "BME280 implausible sample %d.%02d C: adc_t=%d adc_p=%d adc_h=%d (cal T1=%u T2=%d T3=%d)",
                 shown / 100, abs(shown % 100), (int)adc_t, (int)adc_p, (int)adc_h,
                 s_bme.t1, (int)s_bme.t2, (int)s_bme.t3);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

/* ============================= DS1307 =============================== */
static uint8_t bcd_to_dec(uint8_t b) { return (uint8_t)((b >> 4) * 10 + (b & 0x0F)); }

static void rtc_start_if_halted(void)
{
    uint8_t sec = 0;
    I2C_LOCK();
    if (i2c_read_reg(s_rtc_dev, 0x00, &sec, 1) == ESP_OK && (sec & 0x80)) {
        i2c_write_reg(s_rtc_dev, 0x00, (uint8_t)(sec & 0x7F));  // clear Clock Halt
        ESP_LOGW(TAG, "DS1307 oscillator was halted, started it");
    }
    I2C_UNLOCK();
}

static uint8_t dec_to_bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// The RTC stores whatever you write to it: pass local time, the display shows it unchanged.
static esp_err_t rtc_set_time(const rtc_time_w_t *t)
{
    static const int month_offset[12] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    const int y = t->year - (t->month < 3);
    const int dow = (y + y / 4 - y / 100 + y / 400 + month_offset[t->month - 1] + t->day) % 7; // 0 = Sunday

    const uint8_t d[8] = {
        0x00,                                  // start at register 0x00, Clock Halt cleared
        dec_to_bcd(t->second),
        dec_to_bcd(t->minute),
        dec_to_bcd(t->hour),                   // bit 6 = 0 -> 24-hour mode
        dec_to_bcd((uint8_t)(dow == 0 ? 7 : dow)),  // 1 = Monday .. 7 = Sunday
        dec_to_bcd(t->day),
        dec_to_bcd(t->month),
        dec_to_bcd((uint8_t)(t->year - 2000)),
    };
    I2C_LOCK();
    const esp_err_t err = i2c_master_transmit(s_rtc_dev, d, sizeof(d), 1000);
    I2C_UNLOCK();
    return err;
}

static esp_err_t rtc_read(rtc_time_t *t)
{
    uint8_t d[7];
    I2C_LOCK();
    esp_err_t err = i2c_read_reg(s_rtc_dev, 0x00, d, sizeof(d));
    I2C_UNLOCK();
    if (err != ESP_OK) return err;

    t->second = bcd_to_dec((uint8_t)(d[0] & 0x7F));
    t->minute = bcd_to_dec((uint8_t)(d[1] & 0x7F));
    int hour;
    if (d[2] & 0x40) {                  // DS1307 defaults to 12-hour mode after power-on
        hour = bcd_to_dec((uint8_t)(d[2] & 0x1F));
        if (d[2] & 0x20) {
            if (hour < 12) hour += 12;  // PM
        } else if (hour == 12) {
            hour = 0;                   // 12 AM = midnight
        }
    } else {
        hour = bcd_to_dec((uint8_t)(d[2] & 0x3F));  // 24-hour mode: bit 5 is part of the hour
    }
    t->hour = (uint8_t)hour;
    t->day    = bcd_to_dec((uint8_t)(d[4] & 0x3F));
    t->month  = bcd_to_dec((uint8_t)(d[5] & 0x1F));
    t->year   = bcd_to_dec(d[6]);
    return ESP_OK;
}

static time_t record_epoch(const log_record_t *r)
{
    static const int mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    const int year = (int)r->year;
    const bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    int days = 0;
    for (int y = 1970; y < year; y++) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }
    for (int m = 1; m < r->month && m <= 12; m++) {
        days += mdays[m - 1];
        if (m == 2 && leap) days++;
    }
    days += (r->day > 0 ? r->day - 1 : 0);
    return (time_t)days * 86400 + (time_t)r->hour * 3600 + (time_t)r->minute * 60 + r->second;
}

/* ============================= SSD1306 ============================== */
// s_oled_fb[page][column]: bit0 = top pixel of that 8-row page block.
static void oled_text(int col, int row, const char *s)
{
    for (int i = 0; s[i] != '\0' && col + i < OLED_COLS; i++) {
        const uint8_t *g = font8x8_basic[(uint8_t)s[i] & 0x7F];
        for (int r = 0; r < 8; r++) {
            for (int b = 0; b < 8; b++) {
                if (g[r] & (1 << b)) {
                    s_oled_fb[row * OLED_WIDTH + (col + i) * 8 + b] |= (uint8_t)(1u << r);
                }
            }
        }
    }
}

static void oled_render(const display_snapshot_t *snap)
{
    char line[32];   // oled_text clips to the 16 visible columns
    memset(s_oled_fb, 0, sizeof(s_oled_fb));

    if (!snap->sensor_ok) {
        oled_text(0, 3, "NO SENSOR DATA");
    } else {
        snprintf(line, sizeof(line), "%04u-%02u-%02u", snap->record.year, snap->record.month, snap->record.day);
        oled_text(0, 0, line);
        snprintf(line, sizeof(line), "%02u:%02u:%02u", snap->record.hour, snap->record.minute, snap->record.second);
        oled_text(0, 1, line);

        const int t = snap->record.temperature_c_x100;
        snprintf(line, sizeof(line), "T %d.%02d C", t / 100, abs(t % 100));
        oled_text(0, 2, line);
        snprintf(line, sizeof(line), "H %u.%02u %%",
                 snap->record.humidity_pct_x100 / 100, snap->record.humidity_pct_x100 % 100);
        oled_text(0, 3, line);
        snprintf(line, sizeof(line), "P %u mmHg", snap->record.pressure_mmhg);
        oled_text(0, 4, line);
    }

    if (snap->partition_full) {
        oled_text(0, 6, "LOG FULL");
    } else {
        snprintf(line, sizeof(line), "LOG %s %u/%u",
                 snap->logging_enabled ? "ON" : "OFF",
                 (unsigned)snap->buffer_used, (unsigned)FLASH_SECTOR_SIZE);
        oled_text(0, 6, line);
    }

    I2C_LOCK();
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, s_oled_fb);
    I2C_UNLOCK();
}

/* ============================== TASKS =============================== */
static void sensor_task(void *arg)
{
    (void)arg;
    log_record_t rec;
    rtc_time_t now;

    while (1) {
        if (rtc_read(&now) != ESP_OK) {
            ESP_LOGW(TAG, "DS1307 read failed");
        } else {
            rec.year = (uint16_t)(2000 + now.year);
            rec.month = now.month;
            rec.day = now.day;
            rec.hour = now.hour;
            rec.minute = now.minute;
            rec.second = now.second;

            if (bme280_read(&rec.temperature_c_x100, &rec.humidity_pct_x100, &rec.pressure_mmhg) != ESP_OK) {
                ESP_LOGW(TAG, "BME280 read failed");
                static uint8_t retry;                       // self-heal if init failed at boot
                if (++retry >= 10) { retry = 0; bme280_init(); }
            } else if (xQueueSend(s_sensor_q, &rec, 0) != pdTRUE) {
                ESP_LOGW(TAG, "sensor_queue full");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(WAKE_PERIOD_MS));
    }
}

static void display_task(void *arg)
{
    (void)arg;
    display_snapshot_t snap;

    while (1) {
        if (xQueueReceive(s_display_q, &snap, portMAX_DELAY) != pdTRUE) continue;
        oled_render(&snap);
    }
}

static void flash_writer_task(void *arg)
{
    (void)arg;
    flash_req_t req;

    while (1) {
        if (xQueueReceive(s_flash_q, &req, portMAX_DELAY) != pdTRUE) continue;

        flash_result_t res = FLASH_OK;
        if (s_flash_offset + FLASH_SECTOR_SIZE > s_log_part->size) {
            res = FLASH_FULL;
        } else {
            esp_err_t err = esp_partition_erase_range(s_log_part, s_flash_offset, FLASH_SECTOR_SIZE);
            if (err == ESP_OK) err = esp_partition_write(s_log_part, s_flash_offset, req.data, req.len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "flash write failed: %s", esp_err_to_name(err));
                res = FLASH_ERR;
            } else {
                s_last_sector_records = (uint16_t)(req.len / sizeof(log_record_t));
                s_flash_offset += FLASH_SECTOR_SIZE;
                if (s_flash_offset + FLASH_SECTOR_SIZE > s_log_part->size) res = FLASH_FULL;
            }
        }
        xQueueSend(s_flash_result_q, &res, portMAX_DELAY);
    }
}

// Hand the whole RAM buffer to the Flash Writer Task and wait for the result.
// Safe: the buffer is not touched again until the writer replies.
static bool flush_log_buffer(void)
{
    if (s_log_buf_len == 0) {
        printf("flush: buffer is empty\n");
        return true;
    }

    const flash_req_t req = { .data = s_log_buf, .len = s_log_buf_len };
    xQueueSend(s_flash_q, &req, portMAX_DELAY);

    flash_result_t res;
    xQueueReceive(s_flash_result_q, &res, portMAX_DELAY);

    if (res == FLASH_FULL) {
        printf("flash partition is full (%u of %u bytes used) - logging stopped\n",
               (unsigned)s_flash_offset, (unsigned)s_log_part->size);
        return false;
    }
    if (res != FLASH_OK) {
        printf("flush: flash write error\n");
        return false;
    }

    printf("flush: %u records written at offset %u\n",
           (unsigned)(s_log_buf_len / sizeof(log_record_t)), (unsigned)s_flash_offset);
    s_log_buf_len = 0;
    return true;
}

// dump reads only the last written sector: no metadata is stored in flash on purpose.
static void dump_records(int32_t count)
{
    if (s_flash_offset == 0) {
        printf("dump: nothing logged yet\n");
        return;
    }
    if (count <= 0) count = DUMP_DEFAULT_RECORDS;

    static uint8_t sector[FLASH_SECTOR_SIZE];
    const size_t offset = s_flash_offset - FLASH_SECTOR_SIZE;
    if (esp_partition_read(s_log_part, offset, sector, sizeof(sector)) != ESP_OK) {
        printf("dump: read failed\n");
        return;
    }

    int total = s_last_sector_records;
    if (total <= 0 || total > (int)RECORDS_PER_SECTOR) total = (int)RECORDS_PER_SECTOR;
    int start = total - (int)count;
    if (start < 0) start = 0;

    printf("last %d record(s) of sector @0x%x:\n", total - start, (unsigned)offset);
    for (int i = start; i < total; i++) {
        log_record_t r;
        memcpy(&r, sector + i * sizeof(log_record_t), sizeof(r));
        printf("%04u-%02u-%02u %02u:%02u:%02u  T=%d.%02uC  H=%u.%02u%%  P=%ummHg\n",
               r.year, r.month, r.day, r.hour, r.minute, r.second,
               r.temperature_c_x100 / 100, abs(r.temperature_c_x100 % 100),
               r.humidity_pct_x100 / 100, r.humidity_pct_x100 % 100,
               r.pressure_mmhg);
    }
}

static void print_status(bool logging, bool full)
{
    printf("status: log=%s full=%s buffer=%u/%u records_per_sector=%u flash=%u/%u "
           "uptime=%us interval=%umin wake=%ums\n",
           logging ? "on" : "off",
           full ? "yes" : "no",
           (unsigned)s_log_buf_len, (unsigned)sizeof(s_log_buf),
           (unsigned)RECORDS_PER_SECTOR,
           (unsigned)s_flash_offset, (unsigned)s_log_part->size,
           (unsigned)(esp_timer_get_time() / 1000000),
           (unsigned)LOG_INTERVAL_MINUTES, (unsigned)WAKE_PERIOD_MS);
}

static void manager_task(void *arg)
{
    (void)arg;
    bool logging = true;
    bool full = false;
    log_record_t latest = { 0 };
    bool have_data = false;
    time_t last_log = 0;
    bool have_last_log = false;

    while (1) {
        /* 1. UART commands (applied within one wake cycle) */
        uart_command_t cmd;
        while (xQueueReceive(s_cmd_q, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
            case CMD_HELP:
                printf("commands: status | log on | log off | flush | dump [n] | erase | settime YYYY-MM-DD HH:MM:SS | help\n");
                break;

            case CMD_SET_TIME:
                if (rtc_set_time(&cmd.rtc) == ESP_OK) {
                    printf("RTC set to %04d-%02d-%02d %02d:%02d:%02d\n",
                           cmd.rtc.year, cmd.rtc.month, cmd.rtc.day,
                           cmd.rtc.hour, cmd.rtc.minute, cmd.rtc.second);
                } else {
                    printf("settime failed\n");
                }
                break;
            case CMD_STATUS:
                print_status(logging, full);
                break;
            case CMD_LOG_ON:
                if (full) printf("log on: partition is full, run 'erase' first\n");
                else { logging = true; printf("log on\n"); }
                break;
            case CMD_LOG_OFF:
                logging = false;
                printf("log off\n");
                break;
            case CMD_FLUSH:
                if (!flush_log_buffer()) { full = true; logging = false; }
                break;
            case CMD_ERASE:
                s_log_buf_len = 0;
                if (esp_partition_erase_range(s_log_part, 0, s_log_part->size) == ESP_OK) {
                    s_flash_offset = 0;
                    s_last_sector_records = 0;
                    full = false;
                    logging = true;
                    have_last_log = false;
                    printf("erase: log partition cleared (%u bytes)\n", (unsigned)s_log_part->size);
                } else {
                    printf("erase: failed\n");
                }
                break;
            case CMD_DUMP:
                dump_records(cmd.argument);
                break;
            }
        }

        /* 2. Wait for one sensor sample; this blocking wait is what lets the
              IDLE task enter Light Sleep for the rest of the 1 second cycle. */
        log_record_t rec;
        if (xQueueReceive(s_sensor_q, &rec, pdMS_TO_TICKS(WAKE_PERIOD_MS)) == pdTRUE) {
            latest = rec;
            have_data = true;
        }

        /* 3. Log once per LOG_INTERVAL_MINUTES */
        bool logging_now = logging && !full && have_data;
        if (logging_now) {
            const time_t now = record_epoch(&latest);
            if (!have_last_log || now - last_log >= LOG_INTERVAL_MINUTES * 60 || now < last_log) {
                if (s_log_buf_len + sizeof(rec) > sizeof(s_log_buf)) {
                    if (!flush_log_buffer()) { full = true; logging = false; }
                }
                if (!full) {
                    memcpy(s_log_buf + s_log_buf_len, &latest, sizeof(rec));
                    s_log_buf_len += sizeof(rec);
                    last_log = now;
                    have_last_log = true;
                }
            }
        }

        /* 4. Push the current snapshot to the Display Task */
        const display_snapshot_t snap = {
            .record = latest,
            .buffer_used = (uint16_t)s_log_buf_len,
            .logging_enabled = logging,
            .partition_full = full,
            .sensor_ok = have_data,
        };
        if (xQueueSend(s_display_q, &snap, 0) != pdTRUE) {
            ESP_LOGW(TAG, "display_queue full");
        }
    }
}

static void uart_task(void *arg)
{
    (void)arg;
    char line[64];

    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));   // console not ready: do not spin
            continue;
        }
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] == '\0') continue;

        if (strncasecmp(line, "settime", 7) == 0) {   // needs its spaces, so parse it before normalizing
            uart_command_t c = { .type = CMD_SET_TIME };
            int y, mo, d, h, mi, s;
            if (sscanf(line + 7, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) != 6 &&
                sscanf(line + 7, "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &s) != 6) {
                printf("usage: settime YYYY-MM-DD HH:MM:SS\n");
                continue;
            }
            if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || s > 59) {
                printf("settime: value out of range\n");
                continue;
            }
            c.rtc = (rtc_time_w_t){ (int16_t)y, (uint8_t)mo, (uint8_t)d, (uint8_t)h, (uint8_t)mi, (uint8_t)s };
            xQueueSend(s_cmd_q, &c, 0);
            continue;
        }

        char cmd[32];
        int n = 0;
        for (int i = 0; line[i] != '\0' && n < (int)sizeof(cmd) - 1; i++) {
            if (line[i] != ' ') cmd[n++] = (char)tolower((unsigned char)line[i]);
        }
        cmd[n] = '\0';
        if (cmd[0] == '\0') continue;

        uart_command_t c = { 0 };
        if (strcmp(cmd, "help") == 0) c.type = CMD_HELP;
        else if (strcmp(cmd, "status") == 0) c.type = CMD_STATUS;
        else if (strcmp(cmd, "logon") == 0) c.type = CMD_LOG_ON;
        else if (strcmp(cmd, "logoff") == 0) c.type = CMD_LOG_OFF;
        else if (strcmp(cmd, "flush") == 0) c.type = CMD_FLUSH;
        else if (strcmp(cmd, "erase") == 0) c.type = CMD_ERASE;
        else if (strncmp(cmd, "dump", 4) == 0) { c.type = CMD_DUMP; c.argument = atoi(&cmd[4]); }
        else {
            printf("unknown command: %s (type 'help')\n", line);
            continue;
        }
        xQueueSend(s_cmd_q, &c, 0);
    }
}

/* ============================= SETUP ================================ */
static void init_i2c_bus(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &s_bus));

    const i2c_device_config_t bme = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &bme, &s_bme_dev));

    const i2c_device_config_t rtc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS1307_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &rtc, &s_rtc_dev));
}

static void init_oled(void)
{
    const esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = SSD1306_ADDR,
        .scl_speed_hz = 400000,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(s_bus, &io_config, &io));

    esp_lcd_panel_ssd1306_config_t ssd1306_config = { .height = OLED_HEIGHT };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .bits_per_pixel = 1,
        .vendor_config = &ssd1306_config,
    };
    // If the module shows the image rotated, add esp_lcd_panel_mirror(s_panel, true, true) here.
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io, &panel_config, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
}

void app_main(void)
{
    s_i2c_mutex = xSemaphoreCreateMutex();
    init_i2c_bus();
    init_oled();

    // Missing BME280 must not kill the whole system: the display then shows NO SENSOR DATA.
    const esp_err_t bme_err = bme280_init();
    if (bme_err != ESP_OK) {
        ESP_LOGW(TAG, "BME280 not ready: %s", esp_err_to_name(bme_err));
    }
    rtc_start_if_halted();

    s_log_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
                                          LOG_PARTITION_LABEL);
    if (s_log_part == NULL) {
        ESP_LOGE(TAG, "partition '%s' not found - check partitions.csv", LOG_PARTITION_LABEL);
        return;
    }
    ESP_LOGI(TAG, "log partition '%s': %u bytes, %u records per sector",
             s_log_part->label, (unsigned)s_log_part->size, (unsigned)RECORDS_PER_SECTOR);

    s_sensor_q = xQueueCreate(4, sizeof(log_record_t));
    s_cmd_q = xQueueCreate(4, sizeof(uart_command_t));
    s_display_q = xQueueCreate(2, sizeof(display_snapshot_t));
    s_flash_q = xQueueCreate(1, sizeof(flash_req_t));
    s_flash_result_q = xQueueCreate(1, sizeof(flash_result_t));

    xTaskCreate(sensor_task, "sensor", 4096, NULL, 6, NULL);
    xTaskCreate(manager_task, "manager", 4096, NULL, 5, NULL);
    xTaskCreate(display_task, "display", 4096, NULL, 4, NULL);
    xTaskCreate(uart_task, "uart", 4096, NULL, 3, NULL);
    xTaskCreate(flash_writer_task, "flash", 4096, NULL, 3, NULL);

    printf("Power-aware logger ready. Type 'help' for commands.\n");
}
