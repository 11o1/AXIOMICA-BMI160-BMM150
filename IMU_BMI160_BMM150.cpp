/**
 * @file IMU_BMI160_BMM150.cpp
 * @brief Реализация библиотеки для работы с IMU системой (BMI160 + BMM150)
 * 
 * Эта библиотека предоставляет удобный интерфейс для работы с 9-осевой IMU системой,
 * состоящей из датчиков BMI160 (акселерометр и гироскоп) и BMM150 (магнитометр).
 * 
 * Основные функции:
 * - Автоматическое обнаружение и инициализация датчиков
 * - Поддержка двух режимов подключения BMM150:
 *   * Прямое подключение к шине I2C (PRIMARY)
 *   * Подключение через вторичный интерфейс BMI160 (SECONDARY)
 * - Настройка диапазонов измерений акселерометра и гироскопа
 * - Управление режимами работы датчиков
 * 
 * Важные особенности:
 * 1. Полное сканирование I2C шины для поиска BMM150
 * 2. Гибкая обработка частичного подключения датчиков
 * 3. Подробная диагностика через Serial (при включенной отладке)
 * 4. Поддержка работы только с доступными датчиками
 * 
 * @author Bosch Sensortec + Custom
 * @date 2025-10-11
 * @version 1.2
 * 
 * Документация:
 * - BMI160: BMM150 DOC012143196.pdf
 * - BMM150: BMM150 DOC012143196.pdf
 */

#include "IMU_BMI160_BMM150.h"

// === АДРЕСА И РЕГИСТРЫ BMI160 ===
#define BMI160_ADDR_68 0x68
#define BMI160_ADDR_69 0x69

// Регистры BMI160
#define BMI160_CHIP_ID      0x00
#define BMI160_CMD          0x7E
#define BMI160_ACC_CONF     0x40
#define BMI160_ACC_RANGE    0x41
#define BMI160_GYR_CONF     0x42
#define BMI160_GYR_RANGE    0x43
#define BMI160_MAG_IF_0     0x4B
#define BMI160_MAG_IF_1     0x4C
#define BMI160_MAG_IF_2     0x4D
#define BMI160_MAG_IF_3     0x4E
#define BMI160_MAG_IF_4     0x4F
#define BMI160_MAG_CONF     0x44
#define BMI160_DATA_0       0x04
#define BMI160_STATUS       0x1B
#define BMI160_BMM150_IF    0x7D
#define BMI160_PMU_STATUS   0x03

// Команды BMI160
#define BMI160_CMD_SOFTRESET  0xB6
#define BMI160_CMD_ACC_NORMAL 0x11
#define BMI160_CMD_GYR_NORMAL 0x15
#define BMI160_CMD_MAG_NORMAL 0x19

// Регистры BMM150
#define BMM150_CHIP_ID      0x40
#define BMM150_POWER        0x4B
#define BMM150_OPMODE       0x4C
#define BMM150_DATA_X       0x42

// Команды BMM150
#define BMM150_FORCED_MODE  0x02

// Время ожидания после инициализации (мс)
#define INIT_DELAY 100

// Максимальная частота для акселерометра (Гц)
#define MAX_ACC_FREQUENCY 1600

// Максимальная частота для гироскопа (Гц)
#define MAX_GYR_FREQUENCY 3200

// Максимальная частота для магнитометра (Гц)
#define MAX_MAG_FREQUENCY 100

// === СТАТИЧЕСКИЕ ПЕРЕМЕННЫЕ ===
static uint8_t bmi160_addr = 0;
static uint8_t bmm150_addr = 0;
static MagMode mag_mode = NONE;
static SensorConfig config = {
    .acc_odr = 0x28,    // 25 Hz (значение по умолчанию)
    .acc_range = 0x05,  // ±4g (значение по умолчанию)
    .gyr_odr = 0x28,    // 25 Hz (значение по умолчанию)
    .gyr_range = 0x00,  // ±2000°/s (значение по умолчанию)
};
static bool initialized = false;
float ACC_LSB = 8192.0f;  // Значение по умолчанию для ±4g (8192 LSB/g)
float GYR_LSB = 16.384f;  // Значение по умолчанию для ±2000°/s (16.384 LSB/°/s)

// === ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ===

/**
 * @brief Проверяет наличие устройства по указанному адресу I2C
 * 
 * @param addr Адрес устройства
 * @param chip_id Указатель на переменную для сохранения Chip ID (опционально)
 * @param reg Регистр для чтения (по умолчанию 0x00)
 * @return true если устройство существует, false в противном случае
 * 
 * Функция пытается прочитать байт по указанному регистру.
 * Если chip_id не NULL, функция пытается прочитать и сохранить Chip ID.
 * 
 * @note Используется для обнаружения датчиков на шине I2C
 */
static bool i2c_device_exists(uint8_t addr, uint8_t* chip_id, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    
    if (chip_id) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            return false;
        }
        
        unsigned long start = millis();
        uint8_t received = 0;
        while (millis() - start < 5 && received == 0) {
            received = Wire.requestFrom(addr, (uint8_t)1);
        }
        
        if (received == 1) {
            *chip_id = Wire.read();
            return true;
        }
        return false;
    }
    
    return true;
}

/**
 * @brief Безопасная запись в регистр I2C
 * 
 * @param addr Адрес устройства
 * @param reg Регистр для записи
 * @param val Значение для записи
 * @return true если запись прошла успешно, false в случае ошибки
 * 
 * Функция обеспечивает надежную запись в регистр с проверкой завершения
 * 
 * @note Используется для настройки регистров датчиков
 */
static bool i2c_safe_write(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission(true) == 0);
}

/**
 * @brief Безопасное чтение из регистра I2C
 * 
 * @param addr Адрес устройства
 * @param reg Регистр для чтения
 * @param buf Буфер для сохранения прочитанных данных
 * @param len Длина данных для чтения
 * @return true если чтение прошло успешно, false в случае ошибки
 * 
 * Функция проверяет наличие устройства и пытается прочитать данные
 * с таймаутом для ожидания ответа
 * 
 * @note Используется для чтения данных с датчиков
 */
static bool i2c_safe_read(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!i2c_device_exists(addr, nullptr, 0x00)) {
        return false;
    }

    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    unsigned long start = millis();
    uint8_t received = 0;
    while (millis() - start < 10 && received < len) {
        received = Wire.requestFrom(addr, len);
    }

    if (received != len) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

/**
 * @brief Обновляет коэффициенты преобразования в зависимости от текущих настроек
 * 
 * Функция пересчитывает коэффициенты преобразования (ACC_LSB и GYR_LSB)
 * на основе текущих настроек диапазона акселерометра и гироскопа
 * 
 * @note Вызывается автоматически при изменении диапазона измерений
 */
static void update_conversion_factors() {
    switch (config.acc_range) {
        case 0x03: ACC_LSB = 16384.0f; break;
        case 0x05: ACC_LSB = 8192.0f;  break;
        case 0x08: ACC_LSB = 4096.0f;  break;
        case 0x0C: ACC_LSB = 2048.0f;  break;
    }
    switch (config.gyr_range) {
        case 0x00: GYR_LSB = 16.384f; break;
        case 0x01: GYR_LSB = 32.768f; break;
        case 0x02: GYR_LSB = 65.536f; break;
        case 0x03: GYR_LSB = 131.072f; break;
        case 0x04: GYR_LSB = 262.144f; break;
    }
}

/**
 * @brief Инициализирует BMM150, подключенный напрямую к шине I2C
 * 
 * @param addr Адрес BMM150 (0x10-0x13)
 * @return true если инициализация прошла успешно, false в случае ошибки
 * 
 * Функция:
 * 1. Включает питание BMM150
 * 2. Проверяет Chip ID (должен быть 0x32)
 * 3. Настраивает BMM150 в нормальный режим работы
 * 
 * @note Требует, чтобы BMM150 был подключен напрямую к шине I2C
 */
static bool init_bmm150_primary(uint8_t addr) {
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.print(F("  → Инициализация BMM150 на основном интерфейсе: 0x"));
    Serial.println(addr, HEX);
#endif

    // Power On
    if (!i2c_safe_write(addr, BMM150_POWER, 0x01)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Ошибка включения питания"));
#endif
        return false;
    }
    delay(20);

    uint8_t chip_id = 0;
    if (!i2c_safe_read(addr, BMM150_CHIP_ID, &chip_id, 1)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Ошибка чтения Chip ID"));
#endif
        return false;
    }

#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.print(F("    Chip ID = 0x"));
    Serial.println(chip_id, HEX);
#endif
    if (chip_id != 0x32) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Некорректный Chip ID"));
#endif
        return false;
    }
    return true;
}

/**
 * @brief Инициализирует BMM150, подключенный через вторичный интерфейс BMI160
 * 
 * @param phys_addr Физический адрес BMM150 (0x10-0x13)
 * @return true если инициализация прошла успешно, false в случае ошибки
 * 
 * Функция настраивает интерфейс BMI160 для взаимодействия с BMM150:
 * 1. Устанавливает адрес BMM150
 * 2. Включает питание BMM150 через интерфейс BMI160
 * 3. Настраивает BMM150 в нормальный режим работы
 * 4. Проверяет готовность данных магнитометра
 * 
 * @note Требует, чтобы BMM150 был подключен через BMI160
 * 
 * Важные моменты:
 * - Физический адрес BMM150 должен быть сдвинут вправо на 1 бит
 *   для получения 7-битного адреса вторичного интерфейса
 * - Для работы с BMM150 через вторичный интерфейс BMI160
 *   необходимо правильно настроить все регистры:
 *   * BMI160_MAG_IF_0 - адрес BMM150
 *   * BMI160_MAG_IF_1 - режим работы (запись/чтение)
 *   * BMI160_MAG_IF_2 - адрес данных BMM150 (0x42)
 *   * BMI160_MAG_IF_3 - регистр управления BMM150
 *   * BMI160_MAG_IF_4 - данные для записи
 * - Для нормальной работы BMM150 через вторичный интерфейс
 *   необходимо установить регистр MAG_CONF в 0x0B (10 Гц)
 */
static bool init_bmm150_secondary(uint8_t phys_addr) {
    // Рассчитываем 7-битный адрес для вторичного интерфейса
    uint8_t sec_addr = phys_addr >> 1;
    
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.print(F("  → Инициализация BMM150 через вторичный интерфейс (физический адрес: 0x"));
    Serial.print(phys_addr, HEX);
    Serial.println(F(")"));
    
    Serial.print(F("    Физический адрес 0x"));
    Serial.print(phys_addr, HEX);
    Serial.print(F(" → 7-битный адрес вторичного интерфейса: 0x"));
    Serial.println(sec_addr, HEX);
#endif

    uint8_t chip_id = 0;
    if (!i2c_device_exists(bmi160_addr, &chip_id, BMI160_CHIP_ID)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ BMI160 не отвечает"));
#endif
        return false;
    }

    // 1. Устанавливаем адрес BMM150
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_0, sec_addr)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось установить адрес BMM150"));
#endif
        return false;
    }
    delay(1);

    // 2. Переключаемся в режим записи
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x80)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось переключиться в режим записи"));
#endif
        return false;
    }
    delay(1);

    // 3. Устанавливаем регистр управления (0x4B) для включения питания
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_3, 0x4B)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось установить регистр питания"));
#endif
        return false;
    }
    delay(1);

    // 4. Устанавливаем значение питания (0x01)
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_4, 0x01)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось установить значение питания"));
#endif
        return false;
    }
    delay(1);

    // 5. Применяем команду питания
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x80 | (1 << 7))) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось применить команду питания"));
#endif
        return false;
    }
    delay(50);

    // 6. Проверка статуса питания BMM150
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("    Проверка статуса питания BMM150..."));
#endif
    
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_3, 0x4B)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось установить регистр питания для чтения"));
#endif
        return false;
    }
    delay(1);

    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x00)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось настроить интерфейс для чтения"));
#endif
        return false;
    }
    delay(1);

    uint8_t power_status = 0;
    if (!i2c_safe_read(bmi160_addr, BMI160_MAG_IF_4, &power_status, 1)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось прочитать статус питания"));
#endif
        return false;
    }

#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.print(F("    Статус питания BMM150: 0x"));
    Serial.println(power_status, HEX);
#endif

    if (power_status != 0x01) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Питание BMM150 не включено (ожидаем 0x01)"));
#endif
        return false;
    }

    // 7. Установка нормального режима работы BMM150
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("    Установка нормального режима работы BMM150..."));
#endif

    // Устанавливаем регистр режима (0x4C)
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_3, 0x4C)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось установить регистр режима"));
#endif
        return false;
    }
    delay(1);

    // Значение 0x06 соответствует нормальному режиму работы
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_4, 0x06)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось установить нормальный режим"));
#endif
        return false;
    }
    delay(1);

    // Применяем команду
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x80 | (1 << 7))) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось применить команду режима"));
#endif
        return false;
    }
    delay(50);

    // 8. Настройка MAG_CONF для установки ODR (10 Гц)
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_CONF, 0x0B)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось настроить MAG_CONF"));
#endif
        return false;
    }
    delay(1);

    // 9. Установка адреса данных BMM150 (0x42)
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_2, 0x42)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось установить регистр данных"));
#endif
        return false;
    }
    delay(1);

    // 10. Настройка длины пакета данных (8 байт: X, Y, Z, RHALL)
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x03)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось настроить длину пакета данных"));
#endif
        return false;
    }
    delay(1);

    // 11. Включаем магнитометр
    if (!i2c_safe_write(bmi160_addr, BMI160_CMD, BMI160_CMD_MAG_NORMAL)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("    ❌ Не удалось включить магнитометр"));
#endif
        return false;
    }
    delay(100);

    // 12. Проверка готовности данных
    for (int i = 0; i < 200; i++) {
        uint8_t status = 0;
        if (i2c_safe_read(bmi160_addr, BMI160_STATUS, &status, 1) && (status & (1 << 5))) {
            uint8_t data[8] = {0};
            if (i2c_safe_read(bmi160_addr, BMI160_DATA_0, data, 8)) {
                bool all_zero = true;
                for (int j = 0; j < 8; j++) {
                    if (data[j] != 0) {
                        all_zero = false;
                        break;
                    }
                }
                
                if (all_zero) {
#ifdef IMU_BMI160_BMM150_DEBUG
                    Serial.println(F("    ⚠️ Данные все нулевые - проверьте подключение"));
#endif
                } else {
#ifdef IMU_BMI160_BMM150_DEBUG
                    Serial.println(F("    ✅ Данные не нулевые. Синхронизация работает!"));
#endif
                    return true;
                }
            }
        }
        delay(1);
    }

    // 13. Если данные не готовы, пытаемся перезапустить магнитометр
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("    ⚠️ Таймаут ожидания drdy_mag. Попытка чтения данных..."));
#endif
    uint8_t data[8] = {0};
    if (i2c_safe_read(bmi160_addr, BMI160_DATA_0, data, 8)) {
        bool all_zero = true;
        for (int j = 0; j < 8; j++) {
            if (data[j] != 0) {
                all_zero = false;
                break;
            }
        }
        
        if (all_zero) {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("    ❌ Данные все нулевые - проверьте подключение"));
#endif
            return false;
        } else {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("    ✅ Данные не нулевые. Синхронизация работает!"));
#endif
            return true;
        }
    }

#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("    ❌ Таймаут ожидания drdy_mag или некорректный Chip ID"));
#endif
    return false;
}

/**
 * @brief Отправляет команду Forced Mode для BMM150 через вторичный интерфейс
 * 
 * @param addr Адрес BMM150
 * @return true если команда успешно отправлена, false в случае ошибки
 * 
 * Функция:
 * 1. Активирует BMM150 интерфейс
 * 2. Устанавливает адрес BMM150
 * 3. Переключается в режим записи
 * 4. Устанавливает регистр OPMODE
 * 5. Устанавливает значение Forced Mode (0x02)
 * 6. Применяет команду
 * 7. Дает время на измерение и синхронизацию через вторичный интерфейс
 * 8. Проверяет, что данные готовы
 * 
 * Важные моменты:
 * - Для BMM150 в режиме Forced Mode требуется 1-2 мс на измерение
 * - После отправки команды необходимо подождать, прежде чем читать данные
 * - Если данные не готовы, выполняется попытка перезапуска магнитометра
 * 
 * @note Время ожидания установлено в 2 мс (согласно документации BMM150)
 */
static bool send_forced_mode_secondary(uint8_t addr) {
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("  → Отправка Forced Mode через вторичный интерфейс..."));
#endif

    // 1. Активируем BMM150 интерфейс
    if (!i2c_safe_write(bmi160_addr, BMI160_BMM150_IF, 0x01)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❌ Не удалось активировать BMM150 интерфейс"));
#endif
        return false;
    }
    delay(1);

    // 2. Устанавливаем адрес BMM150
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_0, addr)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❌ Не удалось установить адрес BMM150"));
#endif
        return false;
    }
    delay(1);

    // 3. Переключаемся в режим записи
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x80)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❌ Не удалось переключиться в режим записи"));
#endif
        return false;
    }
    delay(1);

    // 4. Устанавливаем регистр OPMODE
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_3, BMM150_OPMODE)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❌ Не удалось установить регистр OPMODE"));
#endif
        return false;
    }
    delay(1);
    
    // 5. Устанавливаем значение Forced Mode (0x02)
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_4, BMM150_FORCED_MODE)) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❌ Не удалось установить значение Forced Mode"));
#endif
        return false;
    }
    delay(1);
    
    // 6. Применяем команду
    if (!i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x80 | (1 << 7))) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❌ Не удалось применить команду Forced Mode"));
#endif
        return false;
    }
    
    // 7. Даем время на измерение и синхронизацию через вторичный интерфейс
    // Согласно документации BMM150, время ожидания в Forced Mode - 1-2 мс
    delay(2);
    
    // 8. Проверяем, что данные готовы
    for (int i = 0; i < 50; i++) {
        uint8_t status = 0;
        if (i2c_safe_read(bmi160_addr, BMI160_STATUS, &status, 1) && (status & (1 << 5))) {
            return true;
        }
        delay(1);
    }
    
    // Если данные не готовы, пытаемся перезапустить магнитометр
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("    ⚠️ Данные не готовы. Попытка перезапуска магнитометра..."));
#endif
    
    i2c_safe_write(bmi160_addr, BMI160_MAG_IF_3, 0x4B);
    i2c_safe_write(bmi160_addr, BMI160_MAG_IF_4, 0x01);
    i2c_safe_write(bmi160_addr, BMI160_MAG_IF_1, 0x80 | (1 << 7));
    delay(200);
    
    for (int i = 0; i < 50; i++) {
        uint8_t status = 0;
        if (i2c_safe_read(bmi160_addr, BMI160_STATUS, &status, 1) && (status & (1 << 5))) {
            return true;
        }
        delay(1);
    }
    
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("❌ Таймаут ожидания готовности данных после Forced Mode"));
#endif
    return false;
}

/**
 * @brief Читает данные BMM150 в Forced Mode (прямое подключение)
 * 
 * @param mag Массив для хранения значений магнитометра (x, y, z)
 * @param rhall Указатель на переменную для хранения значения RHALL
 * 
 * Функция:
 * 1. Отправляет команду Forced Mode
 * 2. Читает данные с датчика
 * 3. Обрабатывает данные в формате BMM150
 * 
 * Обработка данных:
 * - X и Y оси имеют 13-битное разрешение (смещение 3 бита)
 * - Z ось имеет 14-битное разрешение (смещение 1 бит)
 * - RHALL имеет 16-битное разрешение
 * 
 * @note Используется только для BMM150, подключенного напрямую к шине I2C
 */
static void read_bmm150_forced(int16_t* mag, int16_t* rhall) {
    if (!i2c_safe_write(bmm150_addr, BMM150_OPMODE, BMM150_FORCED_MODE)) {
        mag[0] = mag[1] = mag[2] = 0;
        *rhall = 0;
        return;
    }
    delay(1);

    uint8_t buf[8] = {0};
    if (!i2c_safe_read(bmm150_addr, BMM150_DATA_X, buf, 8)) {
        mag[0] = mag[1] = mag[2] = 0;
        *rhall = 0;
        return;
    }

    mag[0] = (int16_t)((buf[1] << 8) | buf[0]) >> 3;
    mag[1] = (int16_t)((buf[3] << 8) | buf[2]) >> 3;
    mag[2] = (int16_t)((buf[5] << 8) | buf[4]) >> 1;
    *rhall = (int16_t)((buf[7] << 8) | buf[6]);
}

// === ПУБЛИЧНЫЕ ФУНКЦИИ ===

/**
 * @brief Инициализирует IMU систему
 * 
 * @return true если инициализация прошла успешно, false в случае ошибки
 * 
 * Функция выполняет следующие шаги:
 * 1. Поиск BMI160 по адресам 0x68 и 0x69
 * 2. Настройка параметров акселерометра и гироскопа
 * 3. Поиск BMM150:
 *    - Сначала проверяются основные адреса (0x10-0x13)
 *    - Затем проверяется вторичный интерфейс BMI160
 *    - Если BMM150 не найден, выполняется полное сканирование шины I2C (0x00-0x7F)
 * 4. Инициализация магнитометра в зависимости от обнаруженного режима
 * 
 * @note Функция выводит подробный лог инициализации в Serial (если отладка включена)
 */
bool IMU_begin() {
    Serial.begin(115200);
    Wire.begin();

    // Поиск BMI160
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("1. Поиск BMI160 по адресам 0x68 и 0x69..."));
#endif
    uint8_t candidates[] = {BMI160_ADDR_68, BMI160_ADDR_69};
    for (int i = 0; i < 2; i++) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.print(F("  Адрес 0x"));
        Serial.print(candidates[i], HEX);
        Serial.print(F(" → Существует: "));
#endif
        
        uint8_t chip_id = 0;
        bool exists = i2c_device_exists(candidates[i], &chip_id, BMI160_CHIP_ID);
        
        if (exists) {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.print(F("да | Chip ID = 0x"));
            Serial.println(chip_id, HEX);
#endif
        } else {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("нет | Chip ID = N/A"));
#endif
        }
        
        if (exists && chip_id == 0xD1) {
            bmi160_addr = candidates[i];
            break;
        }
    }
    
    if (bmi160_addr == 0) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❌ BMI160 не найден"));
#endif
    } else {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.print(F("✅ BMI160 найден по адресу 0x"));
        Serial.println(bmi160_addr, HEX);
#endif
    }

    // Настройка BMI160
    if (bmi160_addr) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("\n2. Настройка BMI160:"));
#endif
        
        if (i2c_safe_write(bmi160_addr, BMI160_ACC_CONF, config.acc_odr)) 
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("  ACC_CONF задан"));
#endif
        if (i2c_safe_write(bmi160_addr, BMI160_ACC_RANGE, config.acc_range)) 
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("  ACC_RANGE задан"));
#endif
        if (i2c_safe_write(bmi160_addr, BMI160_GYR_CONF, config.gyr_odr)) 
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("  GYR_CONF задан"));
#endif
        if (i2c_safe_write(bmi160_addr, BMI160_GYR_RANGE, config.gyr_range)) 
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("  GYR_RANGE задан"));
#endif
        
        if (i2c_safe_write(bmi160_addr, BMI160_CMD, BMI160_CMD_SOFTRESET)) {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("  Soft Reset"));
#endif
            delay(100);
        }
        
        if (i2c_safe_write(bmi160_addr, BMI160_CMD, BMI160_CMD_ACC_NORMAL)) {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("  ACC включен"));
#endif
            delay(10);
        }
        
        if (i2c_safe_write(bmi160_addr, BMI160_CMD, BMI160_CMD_GYR_NORMAL)) {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("  GYR включен"));
#endif
            delay(10);
        }
        
        update_conversion_factors();
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("  Коэффициенты преобразования обновлены"));
#endif
    }

    // Поиск BMM150 на основном интерфейсе (0x10-0x13)
#ifdef IMU_BMI160_BMM150_DEBUG
    Serial.println(F("\n3. Поиск BMM150 на основном интерфейсе (0x10–0x13):"));
#endif
    for (uint8_t addr = 0x10; addr <= 0x13; addr++) {
        if (init_bmm150_primary(addr)) {
            bmm150_addr = addr;
            mag_mode = PRIMARY;
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.print(F("✅ BMM150 найден на основном интерфейсе: 0x"));
            Serial.println(addr, HEX);
#endif
            break;
        }
    }

    // Поиск BMM150 через вторичный интерфейс
    if (bmi160_addr && !bmm150_addr) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("\n4. Поиск BMM150 на вторичной шине (0x10–0x13):"));
#endif
        
        for (uint8_t addr = 0x10; addr <= 0x13; addr++) {
            if (init_bmm150_secondary(addr)) {
                bmm150_addr = addr;
                mag_mode = SECONDARY;
#ifdef IMU_BMI160_BMM150_DEBUG
                Serial.print(F("✅ BMM150 найден на вторичной шине: 0x"));
                Serial.println(addr, HEX);
#endif
                break;
            }
        }
        
        // 4.2. Если BMM150 не найден, проверяем напрямую
        if (!bmm150_addr) {
#ifdef IMU_BMI160_BMM150_DEBUG
            Serial.println(F("\n4.3. Дополнительная проверка BMM150 напрямую (0x00–0x7F):"));
#endif
            for (uint8_t addr = 0x00; addr <= 0x7F; addr++) {
                uint8_t chip_id = 0;
                if (i2c_device_exists(addr, &chip_id, BMM150_CHIP_ID) && chip_id == 0x32) {
                    bmm150_addr = addr;
                    mag_mode = PRIMARY;
#ifdef IMU_BMI160_BMM150_DEBUG
                    Serial.print(F("✅ BMM150 обнаружен напрямую по адресу: 0x"));
                    Serial.println(addr, HEX);
#endif
                    break;
                }
            }
        }
    }

    if (!bmm150_addr) {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("❗ BMM150 не найден ни на одном интерфейсе"));
#endif
    } else {
#ifdef IMU_BMI160_BMM150_DEBUG
        Serial.println(F("✓ BMM150 успешно обнаружен"));
#endif
        initialized = true;
    }

    return initialized;
}

/**
 * @brief Считывает данные с сенсоров
 * 
 * @param acc Массив для хранения значений акселерометра (x, y, z)
 * @param gyr Массив для хранения значений гироскопа (x, y, z)
 * @param mag Массив для хранения значений магнитометра (x, y, z)
 * @param rhall Указатель на переменную для хранения значения RHALL
 * 
 * Функция автоматически определяет режим работы магнитометра и:
 * - Если магнитометр подключен напрямую (PRIMARY), отправляет команду Forced Mode
 * - Если магнитометр подключен через BMI160 (SECONDARY), управляется через BMI160
 */
void IMU_readData(int16_t *acc, int16_t *gyr, int16_t *mag, int16_t *rhall) {
    // Сбрасываем данные
    acc[0] = acc[1] = acc[2] = 0;
    gyr[0] = gyr[1] = gyr[2] = 0;
    mag[0] = mag[1] = mag[2] = 0;
    *rhall = 0;

    // Чтение данных от BMI160
    if (bmi160_addr) {
        // Отправка Forced Mode при необходимости
        if (mag_mode == SECONDARY) {
            if (send_forced_mode_secondary(bmm150_addr)) {
                delay(1);
            }
        }
        
        uint8_t buf[20] = {0};
        if (i2c_safe_read(bmi160_addr, BMI160_DATA_0, buf, 20)) {
            // Обработка данных акселерометра (16-битные значения)
            acc[0] = (int16_t)(buf[15] << 8) | buf[14];
            acc[1] = (int16_t)(buf[17] << 8) | buf[16];
            acc[2] = (int16_t)(buf[19] << 8) | buf[18];
            
            // Обработка данных гироскопа (16-битные значения)
            gyr[0] = (int16_t)(buf[9]  << 8) | buf[8];
            gyr[1] = (int16_t)(buf[11] << 8) | buf[10];
            gyr[2] = (int16_t)(buf[13] << 8) | buf[12];

            // Обработка данных магнитометра, если подключен через BMI160
            if (mag_mode == SECONDARY) {
                mag[0] = (int16_t)(buf[1] << 8) | buf[0];
                mag[1] = (int16_t)(buf[3] << 8) | buf[2];
                mag[2] = (int16_t)(buf[5] << 8) | buf[4];
                *rhall = (int16_t)(buf[7] << 8) | buf[6];
            }
        }
    }

    // Чтение данных от BMM150 в Forced Mode (если подключен напрямую)
    if (mag_mode == PRIMARY) {
        read_bmm150_forced(mag, rhall);
    }
}

/**
 * @brief Устанавливает диапазон измерений акселерометра
 * 
 * @param range Новое значение диапазона
 * 
 * Функция обновляет внутреннюю конфигурацию и пересчитывает коэффициенты преобразования
 * 
 * @note Значения диапазона:
 *       0x03: ±2g (16384 LSB/g)
 *       0x05: ±4g (8192 LSB/g)
 *       0x08: ±8g (4096 LSB/g)
 *       0x0C: ±16g (2048 LSB/g)
 */
void IMU_setAccelRange(uint8_t range) {
    if (bmi160_addr) {
        i2c_safe_write(bmi160_addr, BMI160_ACC_RANGE, range);
        config.acc_range = range;
        update_conversion_factors();
    }
}

/**
 * @brief Устанавливает диапазон измерений гироскопа
 * 
 * @param range Новое значение диапазона
 * 
 * Функция обновляет внутреннюю конфигурацию и пересчитывает коэффициенты преобразования
 * 
 * @note Значения диапазона:
 *       0x00: ±2000°/s (16.384 LSB/°/s)
 *       0x01: ±1000°/s (32.768 LSB/°/s)
 *       0x02: ±500°/s (65.536 LSB/°/s)
 *       0x03: ±250°/s (131.072 LSB/°/s)
 *       0x04: ±125°/s (262.144 LSB/°/s)
 */
void IMU_setGyroRange(uint8_t range) {
    if (bmi160_addr) {
        i2c_safe_write(bmi160_addr, BMI160_GYR_RANGE, range);
        config.gyr_range = range;
        update_conversion_factors();
    }
}

/**
 * @brief Возвращает текущий режим работы магнитометра
 * 
 * @return MagMode текущий режим работы магнитометра
 * 
 * Возможные значения:
 * - PRIMARY: BMM150 подключен напрямую к шине I2C
 * - SECONDARY: BMM150 подключен через BMI160
 * - NONE: магнитометр не обнаружен
 */
MagMode IMU_getMagMode() {
    return mag_mode;
}

/**
 * @brief Проверяет статус инициализации системы
 * 
 * @return true если система успешно инициализирована, false в противном случае
 */
bool IMU_isInitialized() {
    return initialized;
}

/**
 * @brief Читает данные сенсоров с заданной частотой, усредняя результаты
 * 
 * @param acc Массив для хранения усредненных значений акселерометра (x, y, z)
 * @param gyr Массив для хранения усредненных значений гироскопа (x, y, z)
 * @param mag Массив для хранения усредненных значений магнитометра (x, y, z)
 * @param rhall Указатель на переменную для хранения усредненного значения RHALL
 * @param frequency Частота опроса (Гц)
 * 
 * Функция:
 * 1. Устанавливает максимальную частоту измерений для всех датчиков
 * 2. Считывает данные с максимально возможной частотой
 * 3. Усредняет данные для достижения заданной частоты
 * 4. Обрабатывает возможные различия в скорости работы датчиков
 * 
 * @note Функция автоматически определяет оптимальный режим работы
 * @note Если заданная частота выше возможной, используется максимальная
 */
void IMU_readDataWithFrequency(int16_t *acc, int16_t *gyr, int16_t *mag, int16_t *rhall, float frequency) {
    // Проверка валидности частоты
    if (frequency <= 0) {
        frequency = 10.0f; // Минимальная частота 10 Гц
    }

    // Определяем максимальную доступную частоту
    float max_frequency = MAX_ACC_FREQUENCY;
    if (max_frequency > MAX_GYR_FREQUENCY) {
        max_frequency = MAX_GYR_FREQUENCY;
    }
    if (mag_mode != NONE && max_frequency > MAX_MAG_FREQUENCY) {
        max_frequency = MAX_MAG_FREQUENCY;
    }

    // Если заданная частота выше максимальной, используем максимальную
    if (frequency > max_frequency) {
        frequency = max_frequency;
    }

    // Вычисляем интервал между усреднениями
    unsigned long interval = (unsigned long)(1000.0f / frequency);
    
    // Определяем, сколько раз нужно считать данные для усреднения
    unsigned long samples_per_interval = (unsigned long)(max_frequency / frequency);
    
    // Устанавливаем максимальные частоты для датчиков
    if (bmi160_addr) {
        // Устанавливаем максимальную частоту для акселерометра (1600 Гц)
        i2c_safe_write(bmi160_addr, BMI160_ACC_CONF, 0x0C);
        // Устанавливаем максимальную частоту для гироскопа (3200 Гц)
        i2c_safe_write(bmi160_addr, BMI160_GYR_CONF, 0x0C);
    }
    
    // Для BMM150 максимальная частота 100 Гц
    if (bmm150_addr && mag_mode == PRIMARY) {
        // Здесь нужно установить максимальную частоту для BMM150
        // Зависит от конкретной реализации BMM150
    }

    // Временная метка последнего усреднения
    static unsigned long last_time = 0;
    unsigned long current_time = millis();
    
    // Накопленные значения для усреднения
    static int32_t acc_sum[3] = {0};
    static int32_t gyr_sum[3] = {0};
    static int32_t mag_sum[3] = {0};
    static int32_t rhall_sum = 0;
    static uint32_t samples = 0;
    
    // Считываем данные с максимально возможной частотой
    int16_t acc_raw[3], gyr_raw[3], mag_raw[3];
    int16_t rhall_raw;
    
    // Проверяем, пришло ли время для нового усреднения
    if (current_time - last_time >= interval) {
        // Время пришло - возвращаем усредненные значения
        if (samples > 0) {
            acc[0] = acc_sum[0] / samples;
            acc[1] = acc_sum[1] / samples;
            acc[2] = acc_sum[2] / samples;
            
            gyr[0] = gyr_sum[0] / samples;
            gyr[1] = gyr_sum[1] / samples;
            gyr[2] = gyr_sum[2] / samples;
            
            mag[0] = mag_sum[0] / samples;
            mag[1] = mag_sum[1] / samples;
            mag[2] = mag_sum[2] / samples;
            
            *rhall = rhall_sum / samples;
            
            // Сбрасываем накопленные значения
            acc_sum[0] = acc_sum[1] = acc_sum[2] = 0;
            gyr_sum[0] = gyr_sum[1] = gyr_sum[2] = 0;
            mag_sum[0] = mag_sum[1] = mag_sum[2] = 0;
            rhall_sum = 0;
            samples = 0;
        }
        
        last_time = current_time;
        return;
    }
    
    // Считываем данные
    IMU_readData(acc_raw, gyr_raw, mag_raw, &rhall_raw);
    
    // Накапливаем суммы для усреднения
    acc_sum[0] += acc_raw[0];
    acc_sum[1] += acc_raw[1];
    acc_sum[2] += acc_raw[2];
    
    gyr_sum[0] += gyr_raw[0];
    gyr_sum[1] += gyr_raw[1];
    gyr_sum[2] += gyr_raw[2];
    
    mag_sum[0] += mag_raw[0];
    mag_sum[1] += mag_raw[1];
    mag_sum[2] += mag_raw[2];
    
    rhall_sum += rhall_raw;
    
    // Увеличиваем счетчик выборок
    samples++;
}