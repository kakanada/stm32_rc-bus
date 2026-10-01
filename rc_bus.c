/**
 ******************************************************************************
 * @file    rc_bus.c
 * @brief   Реализация приёма i-BUS/S.BUS/CRSF и исходящей телеметрии CRSF
 *          (см. rc_bus.h).
 * @author  Mechanic
 * @date    01.10.2026
 * @version 0.11
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include "rc_bus.h"

/* ------------------------------------------------------------------------ */
/*  Константы кадров протоколов (внутренние, наружу не нужны)               */
/* ------------------------------------------------------------------------ */

#define RCBUS_IBUS_FRAME_LEN         32U     /* длина кадра целиком, байт */
#define RCBUS_IBUS_HEADER_LEN_BYTE   0x20U   /* байт0: длина кадра (совпадает с RCBUS_IBUS_FRAME_LEN) */
#define RCBUS_IBUS_CMD_CHANNELS      0x40U   /* байт1: команда "данные каналов" */

#define RCBUS_SBUS_FRAME_LEN         25U     /* длина кадра целиком, байт */
#define RCBUS_SBUS_START_BYTE        0x0FU
#define RCBUS_SBUS_END_BYTE_CLASSIC  0x00U   /* классический S.BUS без телеметрии */
#define RCBUS_SBUS_FLAG_CH17         (1U << 0)
#define RCBUS_SBUS_FLAG_CH18         (1U << 1)
#define RCBUS_SBUS_FLAG_FRAME_LOST   (1U << 2)
#define RCBUS_SBUS_FLAG_FAILSAFE     (1U << 3)

#define RCBUS_CRSF_SYNC_BYTE            0xC8U  /* байт0: адрес назначения "Flight Controller" */
#define RCBUS_CRSF_FRAMETYPE_CHANNELS   0x16U  /* байт2: тип кадра RC_CHANNELS_PACKED */
#define RCBUS_CRSF_CHANNELS_PAYLOAD_LEN 22U    /* 16 каналов x 11 бит = 22 байта данных */

/* RCBUS_CRSF_FRAMETYPE_BATTERY (0x08) - см. публичный enum RCBUS_CRSF_FrameType_t в rc_bus.h */
#define RCBUS_CRSF_BATTERY_PAYLOAD_LEN  8U     /* напряжение(2)+ток(2)+ёмкость(3)+остаток(1) */

/* Бит регистра CR3, включающий аппаратный режим Half-Duplex (Single Wire) -
 * стандартное имя из CMSIS-заголовков STM32 (одинаково для F4/H7). См.
 * rcbus_ensure_half_duplex_receiver() - в UART_InitTypeDef нет отдельного
 * поля "half-duplex", поэтому проверяем регистр напрямую. */
#ifndef USART_CR3_HDSEL
#define USART_CR3_HDSEL 0x00000008U
#endif

/* ------------------------------------------------------------------------ */
/*  Обёртка над stm32_logger (см. RC_BUS_LOGGER_ENABLED в rc_bus.h)         */
/* ------------------------------------------------------------------------ */
/* Коды причин RCBUS_Init(): передаются как value в RC_BUS_INIT_FAIL, чтобы
 * не заводить под них ещё код(ы) в logger_codes.h - там код события, здесь
 * только различение причины ВНУТРИ одного события. */
#define RCBUS_INIT_FAIL_BAD_CONFIG      1
#define RCBUS_INIT_FAIL_UART_MISMATCH   2
#define RCBUS_INIT_FAIL_POOL_EXHAUSTED  3
#define RCBUS_INIT_FAIL_RX_START        4

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

static RCBUS_Handle_t s_pool[RCBUS_MAX_INSTANCES];

/* ------------------------------------------------------------------------ */
/*  Внутренние вспомогательные функции                                      */
/* ------------------------------------------------------------------------ */

/**
 * @brief   Контрольная сумма семейства кадров i-BUS (0xFFFF минус сумма байт).
 * @param   buf  буфер кадра
 * @param   len  число байт, участвующих в сумме
 * @return  16-битная контрольная сумма
 */
static uint16_t rcbus_ibus_checksum16(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0U;
    for (uint16_t i = 0U; i < len; i++)
    {
        sum = (uint16_t)(sum + buf[i]);
    }
    return (uint16_t)(0xFFFFU - sum);
}

/**
 * @brief   Проверяет, что байт конца кадра S.BUS - один из встречающихся на
 *          практике (классический либо S.BUS2 с сигналом телеметрии).
 * @param   end_byte  последний байт кадра S.BUS
 * @return  1, если байт распознан; иначе 0
 */
static uint8_t rcbus_is_known_sbus_end_byte(uint8_t end_byte)
{
    return ((end_byte == 0x00U) || (end_byte == 0x04U) || (end_byte == 0x14U) ||
            (end_byte == 0x24U) || (end_byte == 0x34U)) ? 1U : 0U;
}

/**
 * @brief   Ищет свободный слот в пуле либо уже зарегистрированный по huart.
 * @param   huart  UART, для которого ищется слот
 * @return  указатель на слот; NULL, если пул полон и совпадения нет
 */
static RCBUS_Handle_t *rcbus_find_or_alloc_slot(UART_HandleTypeDef *huart)
{
    uint32_t free_index = RCBUS_MAX_INSTANCES;
    uint32_t has_free = 0U;

    for (uint32_t i = 0U; i < RCBUS_MAX_INSTANCES; i++)
    {
        if (s_pool[i].used != 0U)
        {
            if (s_pool[i].config.huart == huart)
            {
                return &s_pool[i]; /* уже зарегистрирован - переиспользуем */
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
        return NULL; /* пул исчерпан */
    }
    return &s_pool[free_index];
}

/**
 * @brief   Проверяет, что настройки huart->Init соответствуют протоколу.
 * @param   config  конфигурация экземпляра (huart и protocol)
 * @return  1, если настройки корректны; иначе 0
 */
static uint8_t rcbus_check_uart_settings(const RCBUS_Config_t *config)
{
    const UART_InitTypeDef *init = &config->huart->Init;

    if (config->protocol == RCBUS_PROTOCOL_IBUS)
    {
        return ((init->BaudRate == 115200U) &&
                (init->WordLength == UART_WORDLENGTH_8B) &&
                (init->Parity == UART_PARITY_NONE) &&
                (init->StopBits == UART_STOPBITS_1)) ? 1U : 0U;
    }

    if (config->protocol == RCBUS_PROTOCOL_CRSF)
    {
        /* CRSF - прямой (неинвертированный) сигнал, как i-BUS, только на
         * другой скорости. Приёмники встречаются и с раздельными RX/TX
         * (обычный асинхронный UART), и с одним общим проводом (аппаратный
         * Half-Duplex) - модуль только принимает, поэтому ему без разницы,
         * какой из двух режимов настроен в CubeMX (HAL_UART_Init() или
         * HAL_HalfDuplex_Init()): в обоих приём через
         * HAL_UARTEx_ReceiveToIdle_DMA() работает одинаково, различается
         * только сама разводка линии. Поэтому здесь проверяются только
         * параметры линии, а не то, включён ли CR3.HDSEL. */
        return ((init->BaudRate == 420000U) &&
                (init->WordLength == UART_WORDLENGTH_8B) &&
                (init->Parity == UART_PARITY_NONE) &&
                (init->StopBits == UART_STOPBITS_1)) ? 1U : 0U;
    }

    /* S.BUS: 8 бит данных + чётный бит чётности = 9 бит слова на аппаратном
     * уровне STM32 USART (стандартная особенность HAL - чётность занимает
     * старший бит посылки, поэтому при включённой чётности WordLength всегда
     * "на единицу больше", чем кажется по числу бит полезных данных). */
    return ((init->BaudRate == 100000U) &&
            (init->WordLength == UART_WORDLENGTH_9B) &&
            (init->Parity == UART_PARITY_EVEN) &&
            (init->StopBits == UART_STOPBITS_2)) ? 1U : 0U;
}

/**
 * @brief   Проверяет, включён ли на линии аппаратный режим Half-Duplex.
 * @param   huart  UART экземпляра
 * @return  1, если линия Half-Duplex (Single Wire); 0, если обычная
 *          асинхронная (раздельные RX/TX)
 */
static uint8_t rcbus_is_half_duplex(const UART_HandleTypeDef *huart)
{
    return ((huart->Instance->CR3 & USART_CR3_HDSEL) != 0U) ? 1U : 0U;
}

/**
 * @brief   На линии в режиме Half-Duplex принудительно фиксирует направление
 *          "только приём" (TE=0/RE=1), чтобы собственный передатчик МК не
 *          держал общий провод в состоянии "mark" (push-pull) и не забивал
 *          сигнал от приёмника - вне отправки телеметрии (см.
 *          RCBUS_CRSF_SendBatteryTelemetry()) модуль только принимает. На
 *          обычной асинхронной линии (раздельные RX/TX) не действует - там
 *          TX и RX физически разные пины, TE ни на что не влияет.
 * @param   huart  UART экземпляра
 */
static void rcbus_ensure_half_duplex_receiver(UART_HandleTypeDef *huart)
{
    if (rcbus_is_half_duplex(huart) != 0U)
    {
        (void)HAL_HalfDuplex_EnableReceiver(huart);
    }
}

/**
 * @brief  Заново запускает аппаратный приём следующего кадра (DMA + IDLE).
 * @param  h  хэндл экземпляра
 */
static void rcbus_restart_reception(RCBUS_Handle_t *h)
{
    rcbus_ensure_half_duplex_receiver(h->config.huart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(h->config.huart, h->rx_buffer, RCBUS_RX_BUFFER_LEN);

    /* HAL_UARTEx_ReceiveToIdle_DMA включает прерывание "половина буфера
     * принята" (Half-Transfer), которое нам не нужно (границу кадра ищем по
     * простою линии, а не по заполнению буфера) и приведёт к лишнему, ложному
     * срабатыванию HAL_UARTEx_RxEventCallback() с некорректным Size на
     * середине кадра - отключаем его явно после каждого перезапуска приёма. */
    __HAL_DMA_DISABLE_IT(h->config.huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief   Переводит сырое 11-битное значение канала (S.BUS либо CRSF - у
 *          обоих протоколов один и тот же диапазон и центр) в шкалу "мкс".
 * @param   raw11  сырое значение канала (0..2047, центр 992)
 * @return  значение в единой шкале библиотеки (988..2012, центр 1500)
 */
static uint16_t rcbus_raw11_to_us(uint16_t raw11)
{
    int32_t us = 1500 + (((int32_t)raw11 - 992) * 5) / 8;
    return (uint16_t)us;
}

/**
 * @brief   Контрольная сумма CRSF (CRC-8, полином 0xD5 "DVB-S2", без таблицы).
 * @param   buf  буфер (тип кадра + полезная нагрузка, без адреса/длины/CRC)
 * @param   len  число байт, участвующих в сумме
 * @return  8-битная контрольная сумма
 */
static uint8_t rcbus_crsf_crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0U;
    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((uint8_t)(crc << 1) ^ 0xD5U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief   Разбирает буфер как кадр i-BUS каналов, при успехе заполняет h.
 * @param   h     хэндл экземпляра
 * @param   buf   буфер принятого кадра
 * @param   size  число принятых байт
 * @return  1, если кадр валиден и разобран; иначе 0
 */
static uint8_t rcbus_parse_ibus_frame(RCBUS_Handle_t *h, const uint8_t *buf, uint16_t size)
{
    if (size != RCBUS_IBUS_FRAME_LEN)
    {
        return 0U;
    }
    if ((buf[0] != RCBUS_IBUS_HEADER_LEN_BYTE) || (buf[1] != RCBUS_IBUS_CMD_CHANNELS))
    {
        return 0U;
    }

    uint16_t checksum_received = (uint16_t)((uint16_t)buf[30] | ((uint16_t)buf[31] << 8));
    if (rcbus_ibus_checksum16(buf, RCBUS_IBUS_FRAME_LEN - 2U) != checksum_received)
    {
        return 0U; /* битый кадр - контрольная сумма не сошлась */
    }

    for (uint8_t ch = 0U; ch < RCBUS_IBUS_CHANNEL_COUNT; ch++)
    {
        uint16_t raw = (uint16_t)((uint16_t)buf[2U + (2U * ch)] | ((uint16_t)buf[3U + (2U * ch)] << 8));
        h->channels[ch] = raw; /* i-BUS уже передаёт значение в шкале "как мкс" */
    }
    h->channel_count          = RCBUS_IBUS_CHANNEL_COUNT;
    h->digital_ch17           = 0U; /* у i-BUS дискретных каналов 17/18 нет */
    h->digital_ch18           = 0U;
    h->sbus2_telemetry_signal = 0U; /* у i-BUS этого поля не бывает */
    h->sbus_frame_lost_flag   = 0U; /* у i-BUS нет явного флага в кадре */
    h->sbus_failsafe_flag     = 0U;
    return 1U;
}

/**
 * @brief   Разбирает буфер как кадр S.BUS, при успехе заполняет h.
 *
 *          Распаковка: канал ch занимает биты [11*ch .. 11*ch+10] потока
 *          данных, начинающегося с buf[1]. Читается скользящее 24-битное
 *          окно из очередных 3 байт, из которого вырезаются нужные 11 бит.
 * @param   h     хэндл экземпляра
 * @param   buf   буфер принятого кадра
 * @param   size  число принятых байт
 * @return  1, если кадр валиден и разобран; иначе 0
 */
static uint8_t rcbus_parse_sbus_frame(RCBUS_Handle_t *h, const uint8_t *buf, uint16_t size)
{
    if (size != RCBUS_SBUS_FRAME_LEN)
    {
        return 0U;
    }
    if ((buf[0] != RCBUS_SBUS_START_BYTE) || (rcbus_is_known_sbus_end_byte(buf[24]) == 0U))
    {
        return 0U;
    }

    uint32_t bit_index = 0U;
    for (uint8_t ch = 0U; ch < RCBUS_SBUS_CHANNEL_COUNT; ch++)
    {
        uint32_t byte_index = bit_index / 8U;
        uint32_t bit_offset = bit_index % 8U;
        uint32_t window = (uint32_t)buf[1U + byte_index]
                         | ((uint32_t)buf[2U + byte_index] << 8)
                         | ((uint32_t)buf[3U + byte_index] << 16);
        uint16_t raw11 = (uint16_t)((window >> bit_offset) & 0x07FFU);
        h->channels[ch] = rcbus_raw11_to_us(raw11);
        bit_index += 11U;
    }

    uint8_t flags = buf[23];
    h->channel_count          = RCBUS_SBUS_CHANNEL_COUNT;
    h->digital_ch17           = ((flags & RCBUS_SBUS_FLAG_CH17) != 0U) ? 1U : 0U;
    h->digital_ch18           = ((flags & RCBUS_SBUS_FLAG_CH18) != 0U) ? 1U : 0U;
    h->sbus_frame_lost_flag   = ((flags & RCBUS_SBUS_FLAG_FRAME_LOST) != 0U) ? 1U : 0U;
    h->sbus_failsafe_flag     = ((flags & RCBUS_SBUS_FLAG_FAILSAFE) != 0U) ? 1U : 0U;
    h->sbus2_telemetry_signal = (buf[24] != RCBUS_SBUS_END_BYTE_CLASSIC) ? 1U : 0U;
    return 1U;
}

/**
 * @brief   Разбирает буфер как кадр CRSF, при успехе (только для кадра
 *          RC_CHANNELS_PACKED) заполняет h. Приёмник CRSF шлёт на той же
 *          линии и другие типы кадров (например, LINK_STATISTICS) - они
 *          не являются ошибкой линии, но и не кадр каналов.
 * @param   h     хэндл экземпляра
 * @param   buf   буфер принятого кадра
 * @param   size  число принятых байт
 * @return  1, если это валидный кадр каналов (channels обновлены); 2, если
 *          это валидный кадр CRSF другого типа (не ошибка, но не каналы);
 *          0, если кадр битый/не распознан
 */
static uint8_t rcbus_parse_crsf_frame(RCBUS_Handle_t *h, const uint8_t *buf, uint16_t size)
{
    if ((size < 4U) || (buf[0] != RCBUS_CRSF_SYNC_BYTE))
    {
        return 0U;
    }

    uint16_t frame_len = buf[1]; /* тип + данные + CRC, без адреса и самой длины */
    if ((frame_len < 2U) || ((uint16_t)(frame_len + 2U) != size))
    {
        return 0U; /* заявленная длина не совпадает с реально принятой */
    }

    uint8_t crc_received = buf[size - 1U];
    if (rcbus_crsf_crc8(&buf[2], (uint16_t)(frame_len - 1U)) != crc_received)
    {
        return 0U; /* битый кадр - контрольная сумма не сошлась */
    }

    if ((buf[2] != RCBUS_CRSF_FRAMETYPE_CHANNELS) ||
        (frame_len != (RCBUS_CRSF_CHANNELS_PAYLOAD_LEN + 2U)))
    {
        return 2U; /* валидный кадр CRSF другого типа - не ошибка, но не каналы */
    }

    const uint8_t *payload = &buf[3];
    uint32_t bit_index = 0U;
    for (uint8_t ch = 0U; ch < RCBUS_CRSF_CHANNEL_COUNT; ch++)
    {
        uint32_t byte_index = bit_index / 8U;
        uint32_t bit_offset = bit_index % 8U;
        uint32_t window = (uint32_t)payload[byte_index]
                         | ((uint32_t)payload[byte_index + 1U] << 8)
                         | ((uint32_t)payload[byte_index + 2U] << 16);
        uint16_t raw11 = (uint16_t)((window >> bit_offset) & 0x07FFU);
        h->channels[ch] = rcbus_raw11_to_us(raw11);
        bit_index += 11U;
    }

    h->channel_count          = RCBUS_CRSF_CHANNEL_COUNT;
    h->digital_ch17           = 0U; /* у CRSF нет отдельных дискретных каналов 17/18 */
    h->digital_ch18           = 0U;
    h->sbus2_telemetry_signal = 0U; /* поле относится только к S.BUS2 */
    h->sbus_frame_lost_flag   = 0U; /* явного флага в кадре каналов CRSF нет */
    h->sbus_failsafe_flag     = 0U;
    return 1U;
}

/* ------------------------------------------------------------------------ */
/*  Регистрация экземпляра                                                  */
/* ------------------------------------------------------------------------ */

RCBUS_Handle_t *RCBUS_Init(const RCBUS_Config_t *config)
{
    if ((config == NULL) || (config->huart == NULL))
    {
        RCBUS_LOG(LOG_CODE_RC_BUS_INIT_FAIL, 0, RCBUS_INIT_FAIL_BAD_CONFIG);
        return NULL;
    }
    if ((config->protocol != RCBUS_PROTOCOL_IBUS) && (config->protocol != RCBUS_PROTOCOL_SBUS) &&
        (config->protocol != RCBUS_PROTOCOL_CRSF))
    {
        RCBUS_LOG(LOG_CODE_RC_BUS_INIT_FAIL, 0, RCBUS_INIT_FAIL_BAD_CONFIG);
        return NULL;
    }
    if (rcbus_check_uart_settings(config) == 0U)
    {
        RCBUS_LOG(LOG_CODE_RC_BUS_INIT_FAIL, 0, RCBUS_INIT_FAIL_UART_MISMATCH);
        return NULL; /* huart->Init не соответствует выбранному протоколу */
    }

    RCBUS_Handle_t *h = rcbus_find_or_alloc_slot(config->huart);
    if (h == NULL)
    {
        RCBUS_LOG(LOG_CODE_RC_BUS_INIT_FAIL, 0, RCBUS_INIT_FAIL_POOL_EXHAUSTED);
        return NULL; /* пул исчерпан */
    }

    h->config = *config;
    h->index  = (uint8_t)(h - s_pool);
    h->effective_failsafe_timeout_ms = (config->failsafe_timeout_ms != 0U)
        ? config->failsafe_timeout_ms
        : RCBUS_DEFAULT_FAILSAFE_TIMEOUT_MS;

    for (uint8_t ch = 0U; ch < RCBUS_MAX_CHANNELS; ch++)
    {
        h->channels[ch] = 0U;
    }
    if (config->protocol == RCBUS_PROTOCOL_IBUS)
    {
        h->channel_count = RCBUS_IBUS_CHANNEL_COUNT;
    }
    else if (config->protocol == RCBUS_PROTOCOL_CRSF)
    {
        h->channel_count = RCBUS_CRSF_CHANNEL_COUNT;
    }
    else
    {
        h->channel_count = RCBUS_SBUS_CHANNEL_COUNT;
    }
    h->digital_ch17           = 0U;
    h->digital_ch18           = 0U;
    h->sbus2_telemetry_signal = 0U;
    h->sbus_frame_lost_flag   = 0U;
    h->sbus_failsafe_flag     = 0U;
    h->tx_pending             = 0U;
    h->frame_count            = 0U;
    h->error_count            = 0U;
    /* Пока не пришёл первый кадр, таймаут потери связи отсчитывается с
     * момента Init(), а не "с начала времён" (иначе IsFrameLost() был бы
     * ложно 0 сразу после старта, до реального приёма хоть одного кадра). */
    h->last_frame_tick = HAL_GetTick();
    h->used = 1U;

    rcbus_ensure_half_duplex_receiver(config->huart);
    HAL_StatusTypeDef rx_status = HAL_UARTEx_ReceiveToIdle_DMA(config->huart, h->rx_buffer, RCBUS_RX_BUFFER_LEN);
    if ((rx_status != HAL_OK) && (rx_status != HAL_BUSY))
    {
        h->used = 0U; /* откатываем выделение слота - приём не запустился */
        RCBUS_LOG(LOG_CODE_RC_BUS_INIT_FAIL, 0, RCBUS_INIT_FAIL_RX_START);
        return NULL;
    }
    __HAL_DMA_DISABLE_IT(config->huart->hdmarx, DMA_IT_HT);

    return h;
}

/* ------------------------------------------------------------------------ */
/*  Чтение каналов                                                          */
/* ------------------------------------------------------------------------ */

uint16_t RCBUS_GetChannel(const RCBUS_Handle_t *h, uint8_t channel_index)
{
    if ((h == NULL) || (channel_index >= h->channel_count))
    {
        return 0U;
    }
    return h->channels[channel_index];
}

HAL_StatusTypeDef RCBUS_GetChannels(const RCBUS_Handle_t *h, uint16_t *out, uint8_t count)
{
    if ((h == NULL) || (out == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t n = (count < h->channel_count) ? count : h->channel_count;
    for (uint8_t i = 0U; i < n; i++)
    {
        out[i] = h->channels[i];
    }
    return HAL_OK;
}

uint8_t RCBUS_IsFrameLost(const RCBUS_Handle_t *h)
{
    if (h == NULL)
    {
        return 1U;
    }

    uint8_t timed_out = ((uint32_t)(HAL_GetTick() - h->last_frame_tick) >= h->effective_failsafe_timeout_ms) ? 1U : 0U;
    return ((timed_out != 0U) || (h->sbus_frame_lost_flag != 0U)) ? 1U : 0U;
}

uint8_t RCBUS_IsFailsafe(const RCBUS_Handle_t *h)
{
    if (h == NULL)
    {
        return 1U;
    }
    if ((h->config.protocol == RCBUS_PROTOCOL_SBUS) && (h->sbus_failsafe_flag != 0U))
    {
        return 1U;
    }
    /* i-BUS не имеет отдельного флага failsafe в кадре - равносильно потере кадра */
    return RCBUS_IsFrameLost(h);
}

/* ------------------------------------------------------------------------ */
/*  Исходящая телеметрия CRSF                                               */
/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef RCBUS_CRSF_SendFrame(RCBUS_Handle_t *h,
                                        RCBUS_CRSF_FrameType_t frame_type,
                                        const uint8_t *payload,
                                        uint8_t payload_len)
{
    if ((h == NULL) || (h->config.protocol != RCBUS_PROTOCOL_CRSF))
    {
        return HAL_ERROR;
    }
    if ((payload == NULL) && (payload_len != 0U))
    {
        return HAL_ERROR;
    }
    /* Итоговый кадр: адрес(1) + длина(1) + тип(1) + payload + CRC(1). */
    uint16_t frame_len_total = (uint16_t)(payload_len + 4U);
    if (frame_len_total > RCBUS_CRSF_TX_BUFFER_LEN)
    {
        return HAL_ERROR; /* payload не помещается в буфер */
    }
    if (h->tx_pending != 0U)
    {
        return HAL_ERROR; /* предыдущая отправка телеметрии ещё не завершена */
    }

    uint8_t *buf = h->crsf_tx_buffer;
    buf[0] = RCBUS_CRSF_SYNC_BYTE;
    buf[1] = (uint8_t)(payload_len + 2U); /* тип + payload + CRC */
    buf[2] = (uint8_t)frame_type;
    for (uint8_t i = 0U; i < payload_len; i++)
    {
        buf[3U + i] = payload[i];
    }
    buf[3U + payload_len] = rcbus_crsf_crc8(&buf[2], (uint16_t)(payload_len + 1U));

    h->tx_pending = 1U;
    if (rcbus_is_half_duplex(h->config.huart) != 0U)
    {
        /* Приём и передача делят одну линию - останавливаем ещё активный
         * DMA-приём перед переключением направления на передачу, иначе наш
         * кадр пойдёт поверх незавершённого приёма следующего кадра каналов. */
        (void)HAL_UART_AbortReceive(h->config.huart);
        (void)HAL_HalfDuplex_EnableTransmitter(h->config.huart);
    }
    /* На обычной асинхронной линии (раздельные RX/TX) переключать нечего -
     * приём каналов на RX-пине продолжается независимо от передачи на TX. */
    (void)HAL_UART_Transmit_IT(h->config.huart, h->crsf_tx_buffer, frame_len_total);
    return HAL_OK;
}

HAL_StatusTypeDef RCBUS_CRSF_SendBatteryTelemetry(RCBUS_Handle_t *h,
                                                   uint16_t voltage_mv,
                                                   uint16_t current_ma,
                                                   uint32_t capacity_mah,
                                                   uint8_t remaining_pct)
{
    /* Поля кадра CRSF "датчик батареи" - big-endian, напряжение/ток в
     * единицах 0.1 В / 0.1 А (децивольты/дециамперы), ёмкость 24 бита. */
    uint16_t voltage_dv = (uint16_t)(voltage_mv / 100U);
    uint16_t current_da = (uint16_t)(current_ma / 100U);
    uint32_t capacity   = (capacity_mah > 0x00FFFFFFU) ? 0x00FFFFFFU : capacity_mah;

    uint8_t payload[RCBUS_CRSF_BATTERY_PAYLOAD_LEN];
    payload[0] = (uint8_t)(voltage_dv >> 8);
    payload[1] = (uint8_t)voltage_dv;
    payload[2] = (uint8_t)(current_da >> 8);
    payload[3] = (uint8_t)current_da;
    payload[4] = (uint8_t)(capacity >> 16);
    payload[5] = (uint8_t)(capacity >> 8);
    payload[6] = (uint8_t)capacity;
    payload[7] = remaining_pct;

    return RCBUS_CRSF_SendFrame(h, RCBUS_CRSF_FRAMETYPE_BATTERY, payload, RCBUS_CRSF_BATTERY_PAYLOAD_LEN);
}

/* ------------------------------------------------------------------------ */
/*  Обработчики, вызываемые ИЗ ВАШИХ HAL callback-ов                       */
/* ------------------------------------------------------------------------ */

void RCBUS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint32_t i = 0U; i < RCBUS_MAX_INSTANCES; i++)
    {
        RCBUS_Handle_t *h = &s_pool[i];
        if ((h->used == 0U) || (h->config.huart != huart))
        {
            continue; /* не наш экземпляр/слот пуст - фильтрация по полю хэндла */
        }
        if ((h->tx_pending != 0U) && (rcbus_is_half_duplex(huart) != 0U))
        {
            /* Идёт отправка нашей телеметрии по Half-Duplex линии - приёмник
             * аппаратно отключён (HAL_HalfDuplex_EnableTransmitter), событие
             * не может быть новым кадром - игнорируем без перезапуска приёма
             * (это сделает RCBUS_UART_TxCpltCallback). На обычной асинхронной
             * линии это условие никогда не выполняется - приём независим. */
            continue;
        }

        uint8_t result;
        if (h->config.protocol == RCBUS_PROTOCOL_IBUS)
        {
            result = rcbus_parse_ibus_frame(h, h->rx_buffer, Size);
        }
        else if (h->config.protocol == RCBUS_PROTOCOL_CRSF)
        {
            result = rcbus_parse_crsf_frame(h, h->rx_buffer, Size);
        }
        else
        {
            result = rcbus_parse_sbus_frame(h, h->rx_buffer, Size);
        }

        if (result == 1U)
        {
            h->frame_count++;
            h->last_frame_tick = HAL_GetTick();
        }
        else if (result == 0U)
        {
            h->error_count++;
            RCBUS_LOG_MARK(LOG_CODE_RC_BUS_FRAME_ERROR);
        }
        /* result == 2U (только CRSF) - валидный кадр другого типа, не кадр
         * каналов: не ошибка линии, но и не повод обновлять frame_count/
         * last_frame_tick (см. rcbus_parse_crsf_frame). */

        rcbus_restart_reception(h);
    }
}

void RCBUS_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint32_t i = 0U; i < RCBUS_MAX_INSTANCES; i++)
    {
        RCBUS_Handle_t *h = &s_pool[i];
        if ((h->used == 0U) || (h->config.huart != huart))
        {
            continue;
        }

        h->error_count++;
        RCBUS_LOG(LOG_CODE_RC_BUS_UART_ERROR, h->index, h->error_count);
        h->tx_pending = 0U; /* на случай, если ошибка застала посреди отправки телеметрии */
        /* DMA-приём после ошибки USART (overrun/framing/noise) сам себя не
         * восстанавливает - обязательно останавливаем и перезапускаем, иначе
         * приём каналов "зависает" молча после первой же помехи на линии.
         * rcbus_restart_reception() заодно вернёт направление на приём, если
         * ошибка застала линию Half-Duplex в режиме передачи. */
        (void)HAL_UART_AbortReceive(huart);
        rcbus_restart_reception(h);
    }
}

void RCBUS_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint32_t i = 0U; i < RCBUS_MAX_INSTANCES; i++)
    {
        RCBUS_Handle_t *h = &s_pool[i];
        if ((h->used == 0U) || (h->config.huart != huart) || (h->tx_pending == 0U))
        {
            continue; /* не наш экземпляр, либо это не наша передача завершилась */
        }

        h->tx_pending = 0U;
        if (rcbus_is_half_duplex(huart) != 0U)
        {
            /* Переключаем направление обратно на приём и перезапускаем его -
             * он был остановлен (HAL_UART_AbortReceive) перед передачей. */
            rcbus_restart_reception(h);
        }
        /* На обычной асинхронной линии приём никогда не останавливался -
         * перезапускать нечего, снятого выше tx_pending достаточно. */
    }
}
