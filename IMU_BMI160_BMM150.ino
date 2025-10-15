/**
 * @file IMU_BMI160_BMM150.ino
 * @brief Универсальная 9-осевая система: BMI160 + BMM150 (финальная версия)
 * 
 * Эта программа демонстрирует использование библиотеки IMU_BMI160_BMM150
 * для работы с 9-осевой IMU системой, состоящей из:
 * - BMI160 (акселерометр + гироскоп)
 * - BMM150 (магнитометр)
 * 
 * Программа инициализирует систему, считывает данные с датчиков и выводит
 * их в Serial монитор в формате, совместимом с обработкой в Excel или Python.
 * 
 * Особенность: Программа не останавливается, если BMI160 не найден,
 * но при этом корректно работает с BMM150, подключенным напрямую.
 * 
 * @author Bosch + Custom
 * @date 2025-10-11
 * @version 1.3
 * 
 * @note Для включения отладочного вывода раскомментируйте #define IMU_BMI160_BMM150_DEBUG
 * 
 * Пример вывода:
 * 
 * ✅ IMU инициализирована успешно
 * 24	0	0	0	0	0	0	-49	96	184	26909	0.000	0.000	0.000	0.000	0.000	0.000	-14.700	28.800	55.200
 * 131	0	0	0	0	0	0	-51	100	178	26905	0.000	0.000	0.000	0.000	0.000	0.000	-15.300	30.000	53.400
 * 236	0	0	0	0	0	0	-53	90	181	26909	0.000	0.000	0.000	0.000	0.000	0.000	-15.900	27.000	54.300
 * 
 * Структура вывода:
 * - Время (мс)
 * - Данные акселерометра (сырые значения)
 * - Данные гироскопа (сырые значения)
 * - Данные магнитометра (сырые значения)
 * - Значение RHALL
 * - Данные акселерометра в физических единицах (g)
 * - Данные гироскопа в физических единицах (°/s)
 * - Данные магнитометра в физических единицах (μT)
 */

// Для включения отладочного вывода раскомментируйте следующую строку
// #define IMU_BMI160_BMM150_DEBUG

#include <Wire.h>
#include "IMU_BMI160_BMM150.h"

// Частота опроса данных (Гц)
#define DATA_READ_FREQUENCY 50.0f

void setup() {
    Serial.begin(115200);
    
    // Инициализируем IMU систему
    if (IMU_begin()) {
        Serial.println("✅ IMU инициализирована успешно");
    } else {
        Serial.println("⚠️ IMU частично инициализирована - работает только с доступными датчиками");
    }
}

void loop() {
    int16_t acc_raw[3] = {0}, gyr_raw[3] = {0}, mag_raw[3] = {0};
    int16_t rhall_raw = 0;

    // Считываем данные с сенсоров с заданной частотой
    IMU_readDataWithFrequency(acc_raw, gyr_raw, mag_raw, &rhall_raw, DATA_READ_FREQUENCY);

    // Преобразуем данные в физические единицы
    float acc_si[3] = {
        acc_raw[0] / ACC_LSB,
        acc_raw[1] / ACC_LSB,
        acc_raw[2] / ACC_LSB
    };
    
    float gyr_si[3] = {
        gyr_raw[0] / GYR_LSB,
        gyr_raw[1] / GYR_LSB,
        gyr_raw[2] / GYR_LSB
    };
    
    float mag_si[3] = {
        mag_raw[0] * MAG_LSB_UT,
        mag_raw[1] * MAG_LSB_UT,
        mag_raw[2] * MAG_LSB_UT
    };

    // Выводим данные в формате, удобном для анализа
    Serial.print(millis()); Serial.print("\t");
    Serial.print(acc_raw[0]); Serial.print("\t");
    Serial.print(acc_raw[1]); Serial.print("\t");
    Serial.print(acc_raw[2]); Serial.print("\t");
    Serial.print(gyr_raw[0]); Serial.print("\t");
    Serial.print(gyr_raw[1]); Serial.print("\t");
    Serial.print(gyr_raw[2]); Serial.print("\t");
    Serial.print(mag_raw[0]); Serial.print("\t");
    Serial.print(mag_raw[1]); Serial.print("\t");
    Serial.print(mag_raw[2]); Serial.print("\t");
    Serial.print(rhall_raw); Serial.print("\t"); 
    Serial.print(String(acc_si[0], 3)); Serial.print("\t");
    Serial.print(String(acc_si[1], 3)); Serial.print("\t");
    Serial.print(String(acc_si[2], 3)); Serial.print("\t");
    Serial.print(String(gyr_si[0], 3)); Serial.print("\t");
    Serial.print(String(gyr_si[1], 3)); Serial.print("\t");
    Serial.print(String(gyr_si[2], 3)); Serial.print("\t");
    Serial.print(String(mag_si[0], 3)); Serial.print("\t");
    Serial.print(String(mag_si[1], 3)); Serial.print("\t");
    Serial.println(String(mag_si[2], 3));
    
    // Добавляем небольшую задержку для стабильности
    delay(1);
}