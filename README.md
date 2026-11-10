# Power-Aware Logger with RTOS

Mini-project for **ESP32-S3** using **ESP-IDF v5.5.2**.
The system wakes up every second, reads sensor data, updates the OLED display, and periodically stores compact binary log records into the internal flash memory.

## Project goal

This project demonstrates:

- multitasking in FreeRTOS
- synchronization with Mutex
- queue-based communication between tasks
- power saving with **Light Sleep**
- buffered logging into internal flash
- periodic sensor sampling and display updates

## Hardware

- **ESP32-S3 N16R8**
- **BME280** — temperature, humidity, pressure sensor (I2C `0x76`)
- **DS1306** — real-time clock for timestamp (I2C `0x68`, DS1307-compatible)
- **SSD1306** — OLED display 128×64 (I2C `0x3C`)
- UART terminal for commands

All three devices share one I2C bus: **SDA = GPIO5, SCL = GPIO4** (internal pull-ups enabled).

## Main idea

The microcontroller wakes up every **1 second**.

On each wake cycle it:

1. reads current time from **DS1306**
2. reads temperature, humidity, and pressure from **BME280**
3. updates the **SSD1306** display
4. decides whether it is time to save a log record
5. enters **Light Sleep** again for 1 second

Logging to flash is done **less often** than display updates.
For example, the display is refreshed every second, but logging can happen every **1 minute** or every **5 minutes**, depending on the compile-time setting.

To reduce flash wear, records are first collected in a **RAM buffer**.
When the buffer becomes full, it is written to flash as one sector-sized block.

## RTOS architecture

The project uses **5 FreeRTOS tasks**:

### 1. Sensor Task
Responsible for:
- reading BME280
- reading DS1306
- building one log record
- sending it to `sensor_queue`

### 2. UART Task
Responsible for:
- receiving text commands from UART
- parsing user commands
- sending commands to `command_queue`

### 3. Display Task
Responsible for:
- receiving display snapshots from `display_queue`
- updating SSD1306

### 4. Manager Task
Responsible for:
- coordinating the whole application
- receiving sensor records
- receiving UART commands
- deciding when to append a record to the RAM buffer
- forcing flash flush when the buffer is full
- keeping every task blocked between cycles so the IDLE task can enter Light Sleep

### 5. Flash Writer Task
Responsible for:
- writing one full RAM buffer to flash
- erasing one sector before write
- advancing write offset in the log partition
- stopping logging when the partition becomes full

## Synchronization

### Mutex
The project uses an **I2C Mutex** because multiple devices share the bus:

- BME280
- DS1306
- SSD1306

Only one task may use the I2C bus at a time.

### Queues
The project uses the following queues:

- `sensor_queue` — Sensor Task → Manager Task
- `command_queue` — UART Task → Manager Task
- `display_queue` — Manager Task → Display Task
- `flash_queue` — Manager Task → Flash Writer Task (pointer + length of the RAM buffer,
  not a 4 KB copy; the buffer is not touched again until the writer answers)
- `flash_result_queue` — Flash Writer Task → Manager Task (`OK` / `FULL` / `ERR`)

## Logging format

To save memory, logs are stored in **binary format**, not as text.

Each log record contains:

- year
- month
- day
- hour
- minute
- second
- temperature in **°C × 100**
- humidity in **% × 100**
- pressure in **mmHg**

Example:

- `23.45 °C` → `2345`
- `56.78 %` → `5678`

This is more compact than storing strings and is easier to write to flash efficiently.

With `sizeof(log_record_t) == 14`: **292 records per 4096-byte sector**, and
**74 752 records** (≈ 52 days at 1 record per minute) in the 1 MB `logdata` partition.

## RAM buffer strategy

The project uses:

- **one RAM buffer**
- buffer size = **one flash sector**
- records are appended until the buffer is full
- when full, the whole buffer is written to flash

This reduces the number of flash write/erase operations.

## Flash logging policy

Logs are written into a dedicated internal flash partition called:

`logdata`

Important design decisions:

- **no wear leveling**
- **no metadata stored in flash**
- records are written sector by sector
- when the partition becomes full:
  - logging stops
  - warning is printed through UART
  - warning is shown on the OLED display

## Sleep behavior

The system uses **Light Sleep** with timer wakeup.

Behavior:

- wake up every **1 second**
- perform one measurement/display cycle
- go back to Light Sleep

This demonstrates basic energy-saving behavior while still keeping the system responsive.

`esp_light_sleep_start()` is **not** called from a task — that is not safe while FreeRTOS runs.
Light Sleep is entered by the **FreeRTOS IDLE task** (tickless idle): IDF's `vApplicationSleep()`
computes the idle time, arms a timer wakeup, calls `esp_light_sleep_start()` and fixes the tick
count with `vTaskStepTick()`. Enabled in `sdkconfig.defaults`:

```
CONFIG_PM_ENABLE=y
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP=2
```

This only works because no task busy-waits: every task blocks on a queue or on `vTaskDelay()`.
`CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP` stays off so the I2C bus and the SSD1306 keep
their configuration across Light Sleep.

## UART commands

Supported commands:

- `help` — show command list
- `status` — show current system status
- `flush` — force write current RAM buffer to flash
- `dump [n]` — print the last *n* records (default 20) of the last written sector
- `erase` — erase log partition and clear log buffer
- `log on` — enable logging
- `log off` — disable logging

Parsing ignores spaces and case, so `log on`, `logon` and `LOG ON` are the same command.
`erase` wipes 1 MB of flash, so the display freezes for a second or two while it runs.

> Console caveat: between the 1-second cycles the chip is in Light Sleep. Input typed while
> asleep is buffered by the console peripheral and processed on the next wake, but if you ever
> see dropped characters, set `CONFIG_PM_ENABLE=n` in `sdkconfig.defaults` to disable Light Sleep.

---

## Files

| File | Purpose |
|---|---|
| `src/main.c` | the 5 tasks, sensors, display, flash writer, UART commands |
| `src/font8x8.h` | public-domain 8×8 bitmap font used to draw text on the SSD1306 |
| `partitions.csv` | custom partition table with the 1 MB `logdata` partition |
| `sdkconfig.defaults` | target, 16 MB flash, custom partition table, tickless idle (Light Sleep) |
| `src/CMakeLists.txt` | component registration: `esp_driver_i2c`, `esp_lcd`, `esp_partition`, `esp_timer` |

---

## Build & Flash

### PlatformIO

```bash
pio run -t upload && pio device monitor
```

> `platformio.ini` uses `board = esp32-s3-devkitc-1`; the N16R8V variant is expressed through
> `sdkconfig.defaults` (16 MB flash), because `esp32-s3-devkitc-1-n16r8v` is not a PlatformIO board name.

### ESP-IDF

```bash
idf.py set-target esp32s3 && idf.py build flash monitor
```

Compile-time knobs (top of `src/main.c`):

```c
#define WAKE_PERIOD_MS        1000  // one measurement + display cycle
#define LOG_INTERVAL_MINUTES  1     // log to flash once every N minutes
#define FLASH_SECTOR_SIZE     4096  // RAM log buffer = 1 flash sector
```
