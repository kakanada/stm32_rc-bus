/**
 ******************************************************************************
 * @file    rc_bus_telemetry.c
 * @brief   Реализация ответа телеметрией на шине датчиков i-BUS (см.
 *          rc_bus_telemetry.h).
 * @author  Mechanic
 * @date    19.09.2026
 * @version 0.4
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include "rc_bus_telemetry.h"

/* ------------------------------------------------------------------------ */
/*  Константы протокола (внутренние, наружу не нужны)                       */
/* ------------------------------------------------------------------------ */

#define RCBUS_IBUS_POLL_FRAME_LEN    4U      /* кадр опроса всегда 4 байта */
#define RCBUS_IBUS_CMD_DISCOVER      0x80U   /* "датчик здесь?" -> эхо кадра опроса */
#define RCBUS_IBUS_CMD_TYPE          0x90U   /* "какой тип?" -> тип + размер значения */
#define RCBUS_IBUS_CMD_MEASUREMENT   0xA0U   /* "текущее значение?" -> само значение */
#define RCBUS_IBUS_ADDRESS_MASK      0x0FU
#define RCBUS_IBUS_COMMAND_MASK      0xF0U

/* Бит регистра CR3, включающий аппаратный режим Half-Duplex (Single Wire) -
 * стандартное имя из CMSIS-заголовков STM32 (одинаково для F4/H7). Проверяем
 * его напрямую, т.к. в UART_InitTypeDef нет отдельного поля "half-duplex" -
 * режим включается отдельным вызовом HAL_HalfDuplex_Init() в MX-коде. */
#ifndef USART_CR3_HDSEL
#define USART_CR3_HDSEL 0x00000008U
#endif

/* ------------------------------------------------------------------------ */
/*  Обёртка над stm32_logger (см. RC_BUS_LOGGER_ENABLED в rc_bus.h)         */
/* ------------------------------------------------------------------------ */
#define RCBUS_TELEMETRY_INIT_FAIL_BAD_CONFIG      1
#define RCBUS_TELEMETRY_INIT_FAIL_UART_MISMATCH   2
#define RCBUS_TELEMETRY_INIT_FAIL_POOL_EXHAUSTED  3
#define RCBUS_TELEMETRY_INIT_FAIL_RX_START        4

#ifdef RC_BUS_LOGGER_ENABLED
#define RCBUS_LOG(code, source, value)  LOGGER_Log((code), (uint16_t)(source), (int32_t)(value))
#define RCBUS_LOG_MARK(code)            LOGGER_Mark((code))
#else
#define RCBUS_LOG(code, source, value)  ((void)0)
#define RCBUS_LOG_MARK(code)            ((void)0)
#endif

/* ------------------------------------------------------------------------ */
/*  Статический пул хэндлов (без malloc)                                    */
/* ------------------------------------------------------------------------ */

static RCBUS_TelemetryHandle_t s_telemetry_pool[RCBUS_TELEMETRY_MAX_INSTANCES];

/* ------------------------------------------------------------------------ */
/*  Внутренние вспомогательные функции                                      */
/* ------------------------------------------------------------------------ */

/**
 * @brief   Контрольная сумма кадров i-BUS (0xFFFF минус сумма байт).
 * @param   buf  буфер кадра
 * @param   len  число байт, участвующих в сумме
 * @return  16-битная контрольная сумма
 */
static uint16_t rcbus_telemetry_checksum16(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0U;
    for (uint16_t i = 0U; i < len; i++)
    {
        sum = (uint16_t)(sum + buf[i]);
    }
    return (uint16_t)(0xFFFFU - sum);
}

/**
 * @brief   Ищет свободный слот в пуле либо уже зарегистрированный по huart.
 * @param   huart  UART, для которого ищется слот
 * @return  указатель на слот; NULL, если пул полон и совпадения нет
 */
static RCBUS_TelemetryHandle_t *rcbus_telemetry_find_or_alloc_slot(UART_HandleTypeDef *huart)
{
    uint32_t free_index = RCBUS_TELEMETRY_MAX_INSTANCES;
    uint32_t has_free = 0U;

    for (uint32_t i = 0U; i < RCBUS_TELEMETRY_MAX_INSTANCES; i++)
    {
        if (s_telemetry_pool[i].used != 0U)
        {
            if (s_telemetry_pool[i].config.huart == huart)
            {
                return &s_telemetry_pool[i];
            }
        }
        else if (has_free == 0U)
        {
            free_index = i;
            has_free = 1U;
        }
    }

    if (has_free == 0U)
    {
        return NULL;
    }
    return &s_telemetry_pool[free_index];
}

/**
 * @brief  Заново запускает аппаратный приём следующего кадра опроса.
 * @param  h  хэндл экземпляра
 */
static void rcbus_telemetry_restart_reception(RCBUS_TelemetryHandle_t *h)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(h->config.huart, h->rx_buffer, RCBUS_TELEMETRY_RX_BUFFER_LEN);
    __HAL_DMA_DISABLE_IT(h->config.huart->hdmarx, DMA_IT_HT); /* см. пояснение в rc_bus.c */
}

/**
 * @brief  Формирует и отправляет ответ на опрос по адресу addr командой cmd.
 * @param  h     хэндл экземпляра
 * @param  cmd   код команды опроса (DISCOVER/TYPE/MEASUREMENT)
 * @param  addr  адрес датчика (уже проверено, что зарегистрирован)
 */
static void rcbus_telemetry_send_response(RCBUS_TelemetryHandle_t *h, uint8_t cmd, uint8_t addr)
{
    uint8_t len;

    if (cmd == RCBUS_IBUS_CMD_DISCOVER)
    {
        /* "Я здесь" - эхо того же кадра, которым нас опросили. */
        h->tx_buffer[0] = RCBUS_IBUS_POLL_FRAME_LEN;
        h->tx_buffer[1] = (uint8_t)(cmd | addr);
        len = 4U;
    }
    else if (cmd == RCBUS_IBUS_CMD_TYPE)
    {
        h->tx_buffer[0] = 6U;
        h->tx_buffer[1] = (uint8_t)(cmd | addr);
        h->tx_buffer[2] = h->sensors[addr].type;
        h->tx_buffer[3] = h->sensors[addr].length;
        len = 6U;
    }
    else /* RCBUS_IBUS_CMD_MEASUREMENT */
    {
        uint8_t value_len = h->sensors[addr].length;
        /* Значение передаётся как есть в дополнительном коде (uint->побайтно
         * LE) - так того требует протокол для отрицательных величин (напр.
         * отрицательная температура). */
        uint32_t raw_value = (uint32_t)h->sensors[addr].value;

        h->tx_buffer[0] = (uint8_t)(4U + value_len);
        h->tx_buffer[1] = (uint8_t)(cmd | addr);
        for (uint8_t i = 0U; i < value_len; i++)
        {
            h->tx_buffer[2U + i] = (uint8_t)(raw_value >> (8U * i));
        }
        len = (uint8_t)(4U + value_len);
    }

    uint16_t checksum = rcbus_telemetry_checksum16(h->tx_buffer, (uint16_t)(len - 2U));
    h->tx_buffer[len - 2U] = (uint8_t)(checksum & 0xFFU);
    h->tx_buffer[len - 1U] = (uint8_t)(checksum >> 8);

    h->tx_pending = 1U;
    (void)HAL_HalfDuplex_EnableTransmitter(h->config.huart);
    (void)HAL_UART_Transmit_IT(h->config.huart, h->tx_buffer, len);
}

/**
 * @brief   Проверяет, что huart - 115200 8N1 и настроен в режиме Half-Duplex.
 * @param   huart  проверяемый UART
 * @return  1, если настройки корректны; иначе 0
 */
static uint8_t rcbus_telemetry_check_uart_settings(const UART_HandleTypeDef *huart)
{
    uint8_t line_ok = ((huart->Init.BaudRate == 115200U) &&
                        (huart->Init.WordLength == UART_WORDLENGTH_8B) &&
                        (huart->Init.Parity == UART_PARITY_NONE) &&
                        (huart->Init.StopBits == UART_STOPBITS_1)) ? 1U : 0U;
    uint8_t half_duplex_ok = ((huart->Instance->CR3 & USART_CR3_HDSEL) != 0U) ? 1U : 0U;
    return (uint8_t)(line_ok && half_duplex_ok);
}

/* ------------------------------------------------------------------------ */
/*  Регистрация экземпляра и датчиков                                       */
/* ------------------------------------------------------------------------ */

RCBUS_TelemetryHandle_t *RCBUS_TelemetryInit(const RCBUS_TelemetryConfig_t *config)
{
    if ((config == NULL) || (config->huart == NULL))
    {
        RCBUS_LOG(LOG_CODE_RC_BUS_TELEMETRY_INIT_FAIL, 0, RCBUS_TELEMETRY_INIT_FAIL_BAD_CONFIG);
        return NULL;
    }
    if (rcbus_telemetry_check_uart_settings(config->huart) == 0U)
    {
        RCBUS_LOG(LOG_CODE_RC_BUS_TELEMETRY_INIT_FAIL, 0, RCBUS_TELEMETRY_INIT_FAIL_UART_MISMATCH);
        return NULL; /* не 115200 8N1, либо huart не в режиме Half-Duplex */
    }

    RCBUS_TelemetryHandle_t *h = rcbus_telemetry_find_or_alloc_slot(config->huart);
    if (h == NULL)
    {
        RCBUS_LOG(LOG_CODE_RC_BUS_TELEMETRY_INIT_FAIL, 0, RCBUS_TELEMETRY_INIT_FAIL_POOL_EXHAUSTED);
        return NULL; /* пул исчерпан */
    }

    h->config = *config;
    h->index  = (uint8_t)(h - s_telemetry_pool);
    h->poll_count  = 0U;
    h->error_count = 0U;
    h->tx_pending  = 0U;
    for (uint8_t addr = 0U; addr <= RCBUS_TELEMETRY_MAX_ADDRESS; addr++)
    {
        h->sensors[addr].registered = 0U;
        h->sensors[addr].type       = 0U;
        h->sensors[addr].length     = 0U;
        h->sensors[addr].value      = 0;
    }
    h->used = 1U;

    (void)HAL_HalfDuplex_EnableReceiver(config->huart);
    HAL_StatusTypeDef rx_status = HAL_UARTEx_ReceiveToIdle_DMA(config->huart, h->rx_buffer, RCBUS_TELEMETRY_RX_BUFFER_LEN);
    if ((rx_status != HAL_OK) && (rx_status != HAL_BUSY))
    {
        h->used = 0U;
        RCBUS_LOG(LOG_CODE_RC_BUS_TELEMETRY_INIT_FAIL, 0, RCBUS_TELEMETRY_INIT_FAIL_RX_START);
        return NULL;
    }
    __HAL_DMA_DISABLE_IT(config->huart->hdmarx, DMA_IT_HT);

    return h;
}

HAL_StatusTypeDef RCBUS_TelemetryRegisterSensor(RCBUS_TelemetryHandle_t *h, uint8_t address, uint8_t type, uint8_t length)
{
    if ((h == NULL) || (address < RCBUS_TELEMETRY_MIN_ADDRESS) || (address > RCBUS_TELEMETRY_MAX_ADDRESS))
    {
        return HAL_ERROR;
    }
    if ((length != 2U) && (length != 4U))
    {
        return HAL_ERROR; /* протокол допускает только 2- или 4-байтные значения */
    }

    h->sensors[address].registered = 1U;
    h->sensors[address].type       = type;
    h->sensors[address].length     = length;
    h->sensors[address].value      = 0;
    return HAL_OK;
}

HAL_StatusTypeDef RCBUS_TelemetrySetValue(RCBUS_TelemetryHandle_t *h, uint8_t address, int32_t value)
{
    if ((h == NULL) || (address < RCBUS_TELEMETRY_MIN_ADDRESS) || (address > RCBUS_TELEMETRY_MAX_ADDRESS))
    {
        return HAL_ERROR;
    }
    if (h->sensors[address].registered == 0U)
    {
        return HAL_ERROR;
    }

    /* Запись одного int32_t - атомарна на Cortex-M (одна инструкция STR),
     * поэтому гонка с чтением значения из RxEventCallback (другой контекст -
     * прерывание) безопасна без дополнительной блокировки. */
    h->sensors[address].value = value;
    return HAL_OK;
}

/* ------------------------------------------------------------------------ */
/*  Обработчики, вызываемые ИЗ ВАШИХ HAL callback-ов                       */
/* ------------------------------------------------------------------------ */

void RCBUS_TelemetryUART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint32_t i = 0U; i < RCBUS_TELEMETRY_MAX_INSTANCES; i++)
    {
        RCBUS_TelemetryHandle_t *h = &s_telemetry_pool[i];
        if ((h->used == 0U) || (h->config.huart != huart))
        {
            continue; /* не наш экземпляр/слот пуст */
        }
        if (h->tx_pending != 0U)
        {
            /* Пока идёт передача нашего же ответа, приёмник аппаратно
             * отключён (HAL_HalfDuplex_EnableTransmitter) - если событие всё
             * же пришло, это не может быть новый кадр опроса, игнорируем без
             * перезапуска приёма (это сделает TxCpltCallback). */
            continue;
        }

        uint8_t responding = 0U;

        if ((Size == RCBUS_IBUS_POLL_FRAME_LEN) && (h->rx_buffer[0] == RCBUS_IBUS_POLL_FRAME_LEN))
        {
            uint16_t checksum_received = (uint16_t)((uint16_t)h->rx_buffer[2] | ((uint16_t)h->rx_buffer[3] << 8));
            if (rcbus_telemetry_checksum16(h->rx_buffer, 2U) == checksum_received)
            {
                uint8_t addr = (uint8_t)(h->rx_buffer[1] & RCBUS_IBUS_ADDRESS_MASK);
                uint8_t cmd  = (uint8_t)(h->rx_buffer[1] & RCBUS_IBUS_COMMAND_MASK);
                uint8_t cmd_known = ((cmd == RCBUS_IBUS_CMD_DISCOVER) ||
                                     (cmd == RCBUS_IBUS_CMD_TYPE) ||
                                     (cmd == RCBUS_IBUS_CMD_MEASUREMENT)) ? 1U : 0U;

                if ((addr >= RCBUS_TELEMETRY_MIN_ADDRESS) && (addr <= RCBUS_TELEMETRY_MAX_ADDRESS) &&
                    (h->sensors[addr].registered != 0U) && (cmd_known != 0U))
                {
                    h->poll_count++;
                    rcbus_telemetry_send_response(h, cmd, addr);
                    responding = 1U; /* приём перезапустит TxCpltCallback после ответа */
                }
                /* иначе - опрос чужого адреса или неизвестная команда: это
                 * нормальная ситуация на общей шине, молча игнорируем. */
            }
            else
            {
                h->error_count++; /* контрольная сумма не сошлась - битый кадр опроса */
                RCBUS_LOG_MARK(LOG_CODE_RC_BUS_TELEMETRY_FRAME_ERROR);
            }
        }
        else
        {
            h->error_count++; /* неожиданная длина кадра */
            RCBUS_LOG_MARK(LOG_CODE_RC_BUS_TELEMETRY_FRAME_ERROR);
        }

        if (responding == 0U)
        {
            rcbus_telemetry_restart_reception(h);
        }
    }
}

void RCBUS_TelemetryUART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint32_t i = 0U; i < RCBUS_TELEMETRY_MAX_INSTANCES; i++)
    {
        RCBUS_TelemetryHandle_t *h = &s_telemetry_pool[i];
        if ((h->used == 0U) || (h->config.huart != huart) || (h->tx_pending == 0U))
        {
            continue; /* не наш экземпляр, либо это не наша передача завершилась */
        }

        h->tx_pending = 0U;
        (void)HAL_HalfDuplex_EnableReceiver(huart);
        rcbus_telemetry_restart_reception(h);
    }
}

void RCBUS_TelemetryUART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint32_t i = 0U; i < RCBUS_TELEMETRY_MAX_INSTANCES; i++)
    {
        RCBUS_TelemetryHandle_t *h = &s_telemetry_pool[i];
        if ((h->used == 0U) || (h->config.huart != huart))
        {
            continue;
        }

        h->error_count++;
        RCBUS_LOG(LOG_CODE_RC_BUS_TELEMETRY_UART_ERROR, h->index, h->error_count);
        h->tx_pending = 0U;
        (void)HAL_UART_AbortReceive(huart);
        (void)HAL_HalfDuplex_EnableReceiver(huart); /* на случай, если ошибка застала посреди передачи ответа */
        rcbus_telemetry_restart_reception(h);
    }
}
