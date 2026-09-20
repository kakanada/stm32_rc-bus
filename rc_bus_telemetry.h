/**
 ******************************************************************************
 * @file    rc_bus_telemetry.h
 * @brief   Ответ телеметрией на опрос датчиков по отдельной half-duplex шине
 *          FlySky i-BUS ("SENSOR"). МК выступает виртуальным датчиком.
 * @author  Mechanic
 * @date    19.09.2026
 * @version 0.4
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#ifndef RC_BUS_TELEMETRY_H
#define RC_BUS_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"   /* CubeMX: UART_HandleTypeDef и т.п. */

/* ------------------------------------------------------------------------ */
/*  Конфигурация модуля (define-ы, меняются до включения заголовка)         */
/* ------------------------------------------------------------------------ */

/** Максимальное число одновременно зарегистрированных экземпляров (отдельных
 *  физических half-duplex шин датчиков). */
#ifndef RCBUS_TELEMETRY_MAX_INSTANCES
#define RCBUS_TELEMETRY_MAX_INSTANCES   2U
#endif
#if RCBUS_TELEMETRY_MAX_INSTANCES == 0U
#error "RCBUS_TELEMETRY_MAX_INSTANCES must be at least 1"
#endif

/** Диапазон адресов датчиков на шине i-BUS: так устроен протокол (адрес -
 *  младшие 4 бита байта команды), адрес 0 зарезервирован и не используется. */
#define RCBUS_TELEMETRY_MIN_ADDRESS     1U
#define RCBUS_TELEMETRY_MAX_ADDRESS     15U

/** Единственный код типа датчика, который стабильно (без противоречий между
 *  источниками) документирован как "внутреннее напряжение", мВ, 2 байта -
 *  самый распространённый и часто единственный нужный тип телеметрии. Для
 *  остальных типов передавайте код напрямую (см. "ЧЕГО ЗДЕСЬ НАРОЧНО НЕТ"). */
#define RCBUS_IBUS_SENSOR_TYPE_VOLTAGE  0x00U

/** Буфер приёма кадра опроса - сам кадр всегда ровно 4 байта, запас не нужен,
 *  но буфер должен быть отдельным от tx_buffer (DMA приёма работает
 *  независимо от передачи ответа). */
#define RCBUS_TELEMETRY_RX_BUFFER_LEN   4U

/** Буфер ответа - самый длинный ответ (тип датчика или значение 4 байта) -
 *  не более 4 (заголовок+адрес+2 байта чек-суммы) + 4 (значение) = 8 байт. */
#define RCBUS_TELEMETRY_TX_BUFFER_LEN   8U

/* ------------------------------------------------------------------------ */
/*  Необязательная интеграция с stm32_logger - см. RC_BUS_LOGGER_ENABLED в  */
/*  rc_bus.h (то же самое, тот же define, включает оба модуля разом).       */
/* ------------------------------------------------------------------------ */
#ifdef RC_BUS_LOGGER_ENABLED
#include "logger.h"
#include "logger_codes.h"
#endif

/* ------------------------------------------------------------------------ */
/*  Конфигурация одного экземпляра - заполняется пользователем             */
/* ------------------------------------------------------------------------ */

typedef struct
{
    /** USART/UART шины датчиков. Должен быть уже проинициализирован CubeMX-
     *  кодом ЧЕРЕЗ HAL_HalfDuplex_Init() (Half-Duplex Selection -> Single
     *  Wire в CubeMX), 115200 8N1 - RCBUS_TelemetryInit() проверяет оба
     *  условия и вернёт NULL, если они не выполнены. */
    UART_HandleTypeDef *huart;
} RCBUS_TelemetryConfig_t;

/* ------------------------------------------------------------------------ */
/*  Один виртуальный датчик на шине                                        */
/* ------------------------------------------------------------------------ */

typedef struct
{
    uint8_t registered;  /**< 1, если адрес занят зарегистрированным датчиком */
    uint8_t type;         /**< код типа датчика (сырой байт, см. шапку файла) */
    uint8_t length;       /**< размер значения при передаче: 2 или 4 байта    */
    int32_t value;         /**< текущее значение, устанавливается приложением  */
} RCBUS_TelemetrySensor_t;

/* ------------------------------------------------------------------------ */
/*  Хэндл экземпляра - главная структура API                                */
/* ------------------------------------------------------------------------ */
/*
 * RCBUS_TelemetryInit() возвращает указатель НА ЭТУ структуру (память
 * статическая, живёт всё время работы программы).
 */
typedef struct
{
    /* ---- Публичные поля (можно читать снаружи) ---- */
    RCBUS_TelemetryConfig_t config;                 /* копия того, что передали в Init */

    /** Датчики по адресам [1 .. RCBUS_TELEMETRY_MAX_ADDRESS]; sensors[0] не
     *  используется (адрес 0 зарезервирован протоколом). */
    RCBUS_TelemetrySensor_t sensors[RCBUS_TELEMETRY_MAX_ADDRESS + 1U];

    /** Счётчик опросов, адресованных НАШИМ зарегистрированным датчикам, и
     *  счётчик отброшенных битых кадров опроса - для диагностики. */
    uint32_t poll_count;
    uint32_t error_count;

    /** Позиция в статическом пуле (для справки). */
    uint8_t index;

    /* ---- Внутреннее состояние - не трогать напрямую, только через API ---- */
    uint8_t          used;                                          /* слот занят (бухгалтерия пула) */
    uint8_t          rx_buffer[RCBUS_TELEMETRY_RX_BUFFER_LEN];       /* буфер приёма DMA (кадр опроса) */
    uint8_t          tx_buffer[RCBUS_TELEMETRY_TX_BUFFER_LEN];       /* буфер ответа                   */
    volatile uint8_t tx_pending;    /* идёт передача ответа - приём временно не перезапускаем */
} RCBUS_TelemetryHandle_t;

/* ------------------------------------------------------------------------ */
/*  Регистрация экземпляра и датчиков                                       */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Регистрирует экземпляр на указанной half-duplex шине: проверяет
 *         настройки huart (линия + режим Half-Duplex) и запускает аппаратный
 *         приём опроса. Повторный вызов с тем же huart идемпотентен, но
 *         СБРАСЫВАЕТ регистрации всех датчиков этого экземпляра - вызывайте
 *         RCBUS_TelemetryRegisterSensor() заново после повторного Init().
 * @param  config  заполненная конфигурация
 * @retval указатель на хэндл, либо NULL при ошибке (config некорректен,
 *         huart->Init не 115200/8N1, huart не в режиме Half-Duplex,
 *         исчерпан RCBUS_TELEMETRY_MAX_INSTANCES, либо не запустился приём)
 */
RCBUS_TelemetryHandle_t *RCBUS_TelemetryInit(const RCBUS_TelemetryConfig_t *config);

/**
 * @brief  Регистрирует виртуальный датчик по адресу на шине.
 * @param  h        хэндл экземпляра
 * @param  address  адрес датчика, RCBUS_TELEMETRY_MIN_ADDRESS..RCBUS_TELEMETRY_MAX_ADDRESS
 * @param  type     код типа датчика (см. RCBUS_IBUS_SENSOR_TYPE_VOLTAGE и шапку файла)
 * @param  length   размер значения в байтах: 2 или 4
 * @retval HAL_OK; HAL_ERROR при некорректных параметрах (h == NULL, address
 *         вне диапазона, length не 2 и не 4)
 */
HAL_StatusTypeDef RCBUS_TelemetryRegisterSensor(RCBUS_TelemetryHandle_t *h, uint8_t address, uint8_t type, uint8_t length);

/**
 * @brief  Обновляет текущее значение зарегистрированного датчика - будет
 *         отдано приёмнику при следующем опросе этого адреса.
 * @param  h        хэндл экземпляра
 * @param  address  адрес ранее зарегистрированного датчика
 * @param  value    новое значение (знаковое, в единицах, ожидаемых типом
 *                  датчика - например, для RCBUS_IBUS_SENSOR_TYPE_VOLTAGE это мВ)
 * @retval HAL_OK; HAL_ERROR, если h == NULL, address вне диапазона или датчик
 *         с таким адресом не зарегистрирован
 */
HAL_StatusTypeDef RCBUS_TelemetrySetValue(RCBUS_TelemetryHandle_t *h, uint8_t address, int32_t value);

/* ------------------------------------------------------------------------ */
/*  Обработчики, вызываемые ИЗ ВАШИХ HAL callback-ов                       */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Обработчик окончания приёма кадра опроса - вызывайте из своего
 *         HAL_UARTEx_RxEventCallback(). Если кадр адресован одному из наших
 *         зарегистрированных датчиков - формирует и отправляет ответ
 *         (неблокирующе, HAL_UART_Transmit_IT); иначе тихо игнорирует кадр
 *         (адресован другому датчику на общей шине - это норма) и сразу
 *         перезапускает приём.
 * @param  huart  хэндл UART, пришедший в ваш HAL-колбэк как есть
 * @param  Size   число принятых байт, пришедшее в ваш HAL-колбэк как есть
 */
void RCBUS_TelemetryUART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief  Обработчик завершения передачи ответа - вызывайте из своего
 *         HAL_UART_TxCpltCallback(). Обязателен: переключает линию обратно
 *         на приём (HAL_HalfDuplex_EnableReceiver) и перезапускает приём
 *         следующего опроса - без этого шина "зависнет" в режиме передачи
 *         после первого же ответа.
 * @param  huart  хэндл UART, пришедший в ваш HAL-колбэк как есть
 */
void RCBUS_TelemetryUART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief  Обработчик ошибки UART - вызывайте из своего HAL_UART_ErrorCallback().
 *         Восстанавливает приём (и режим приёмника, если ошибка застала
 *         посреди передачи ответа) после framing/noise/overrun.
 * @param  huart  хэндл UART, пришедший в ваш HAL-колбэк как есть
 */
void RCBUS_TelemetryUART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* RC_BUS_TELEMETRY_H */
