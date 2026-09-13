/**
 ******************************************************************************
 * @file    rc_bus.c
 * @brief   Реализация приёма i-BUS/S.BUS (см. rc_bus.h).
 * @author  Claude
 * @date    13.09.2026
 * @version 0.1
 *
 * @copyright Copyright (c) 2026 Claude.
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
#define RCBUS_SBUS_END_BYTE          0x00U
#define RCBUS_SBUS_FLAG_CH17         (1U << 0)
#define RCBUS_SBUS_FLAG_CH18         (1U << 1)
#define RCBUS_SBUS_FLAG_FRAME_LOST   (1U << 2)
#define RCBUS_SBUS_FLAG_FAILSAFE     (1U << 3)

/* ------------------------------------------------------------------------ */
/*  Статический пул хэндлов (без malloc)                                    */
/* ------------------------------------------------------------------------ */

static RCBUS_Handle_t s_pool[RCBUS_MAX_INSTANCES];

/* ------------------------------------------------------------------------ */
/*  Внутренние вспомогательные функции                                      */
/* ------------------------------------------------------------------------ */

/** Ищет свободный слот в пуле либо уже зарегистрированный по этому huart -
 *  для идемпотентности повторного RCBUS_Init(). NULL, если пул полон и
 *  совпадения не найдено. */
static RCBUS_Handle_t *RCBUS_FindOrAllocSlot(UART_HandleTypeDef *huart)
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

/** Проверяет, что настройки уже проинициализированного huart->Init
 *  соответствуют требованиям выбранного протокола (см. таблицу в rc_bus.h).
 *  Ловит ошибку конфигурации CubeMX на этапе Init(), а не молчаливым потоком
 *  битых кадров в рантайме. */
static uint8_t RCBUS_CheckUartSettings(const RCBUS_Config_t *config)
{
    const UART_InitTypeDef *init = &config->huart->Init;

    if (config->protocol == RCBUS_PROTOCOL_IBUS)
    {
        return ((init->BaudRate == 115200U) &&
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

/** Заново запускает аппаратный приём следующего кадра (DMA + определение
 *  простоя линии). Общая для Init() (первый запуск) и обоих диспетчерских
 *  колбэков (перезапуск после кадра/ошибки). */
static void RCBUS_RestartReception(RCBUS_Handle_t *h)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(h->config.huart, h->rx_buffer, RCBUS_RX_BUFFER_LEN);

    /* HAL_UARTEx_ReceiveToIdle_DMA включает прерывание "половина буфера
     * принята" (Half-Transfer), которое нам не нужно (границу кадра ищем по
     * простою линии, а не по заполнению буфера) и приведёт к лишнему, ложному
     * срабатыванию HAL_UARTEx_RxEventCallback() с некорректным Size на
     * середине кадра - отключаем его явно после каждого перезапуска приёма. */
    __HAL_DMA_DISABLE_IT(h->config.huart->hdmarx, DMA_IT_HT);
}

/** Переводит "сырое" 11-битное значение канала S.BUS (0..2047, центр 992) в
 *  единую шкалу библиотеки - "как ширина импульса в мкс" (988..2012, центр
 *  1500), в которой уже и так находятся значения каналов i-BUS. Формула -
 *  стандартное для индустрии RC линейное преобразование S.BUS<->PWM. */
static uint16_t RCBUS_SBusToUs(uint16_t raw11)
{
    int32_t us = 1500 + (((int32_t)raw11 - 992) * 5) / 8;
    return (uint16_t)us;
}

/** Разбирает буфер как кадр i-BUS (данные каналов, команда 0x40): проверяет
 *  длину, заголовок и контрольную сумму (0xFFFF минус сумма всех байт кадра
 *  кроме самой контрольной суммы), при успехе заполняет channels[]. */
static uint8_t RCBUS_ParseIBusFrame(RCBUS_Handle_t *h, const uint8_t *buf, uint16_t size)
{
    if (size != RCBUS_IBUS_FRAME_LEN)
    {
        return 0U;
    }
    if ((buf[0] != RCBUS_IBUS_HEADER_LEN_BYTE) || (buf[1] != RCBUS_IBUS_CMD_CHANNELS))
    {
        return 0U;
    }

    uint16_t sum = 0U;
    for (uint16_t i = 0U; i < (RCBUS_IBUS_FRAME_LEN - 2U); i++)
    {
        sum = (uint16_t)(sum + buf[i]);
    }
    uint16_t checksum_received = (uint16_t)((uint16_t)buf[30] | ((uint16_t)buf[31] << 8));
    if ((uint16_t)(0xFFFFU - sum) != checksum_received)
    {
        return 0U; /* битый кадр - контрольная сумма не сошлась */
    }

    for (uint8_t ch = 0U; ch < RCBUS_IBUS_CHANNEL_COUNT; ch++)
    {
        uint16_t raw = (uint16_t)((uint16_t)buf[2U + (2U * ch)] | ((uint16_t)buf[3U + (2U * ch)] << 8));
        h->channels[ch] = raw; /* i-BUS уже передаёт значение в шкале "как мкс" */
    }
    h->channel_count        = RCBUS_IBUS_CHANNEL_COUNT;
    h->digital_ch17         = 0U; /* у i-BUS дискретных каналов 17/18 нет */
    h->digital_ch18         = 0U;
    h->sbus_frame_lost_flag = 0U; /* у i-BUS нет явного флага в кадре */
    h->sbus_failsafe_flag   = 0U;
    return 1U;
}

/** Разбирает буфер как кадр S.BUS: проверяет длину, старт- и стоп-байт,
 *  распаковывает 16 каналов по 11 бит (упакованы подряд без выравнивания по
 *  байтам, младший бит вперёд) и байт флагов (CH17/CH18/frame lost/failsafe).
 *
 *  Распаковка: канал ch занимает биты [11*ch .. 11*ch+10] потока данных,
 *  начинающегося с buf[1] (buf[0] - старт-байт, в поток не входит). Чтобы не
 *  писать 16 отдельных строк со сдвигами (легко ошибиться в одной), читаем
 *  скользящее 24-битное окно из очередных 3 байт и вырезаем нужные 11 бит -
 *  окна перекрываются, поэтому один и тот же байт может войти в соседний
 *  канал, что и ожидаемо для плотной побитовой упаковки. */
static uint8_t RCBUS_ParseSBusFrame(RCBUS_Handle_t *h, const uint8_t *buf, uint16_t size)
{
    if (size != RCBUS_SBUS_FRAME_LEN)
    {
        return 0U;
    }
    if ((buf[0] != RCBUS_SBUS_START_BYTE) || (buf[24] != RCBUS_SBUS_END_BYTE))
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
        h->channels[ch] = RCBUS_SBusToUs(raw11);
        bit_index += 11U;
    }

    uint8_t flags = buf[23];
    h->channel_count        = RCBUS_SBUS_CHANNEL_COUNT;
    h->digital_ch17         = ((flags & RCBUS_SBUS_FLAG_CH17) != 0U) ? 1U : 0U;
    h->digital_ch18         = ((flags & RCBUS_SBUS_FLAG_CH18) != 0U) ? 1U : 0U;
    h->sbus_frame_lost_flag = ((flags & RCBUS_SBUS_FLAG_FRAME_LOST) != 0U) ? 1U : 0U;
    h->sbus_failsafe_flag   = ((flags & RCBUS_SBUS_FLAG_FAILSAFE) != 0U) ? 1U : 0U;
    return 1U;
}

/* ------------------------------------------------------------------------ */
/*  Регистрация экземпляра                                                  */
/* ------------------------------------------------------------------------ */

RCBUS_Handle_t *RCBUS_Init(const RCBUS_Config_t *config)
{
    if ((config == NULL) || (config->huart == NULL))
    {
        return NULL;
    }
    if ((config->protocol != RCBUS_PROTOCOL_IBUS) && (config->protocol != RCBUS_PROTOCOL_SBUS))
    {
        return NULL;
    }
    if (RCBUS_CheckUartSettings(config) == 0U)
    {
        return NULL; /* huart->Init не соответствует выбранному протоколу */
    }

    RCBUS_Handle_t *h = RCBUS_FindOrAllocSlot(config->huart);
    if (h == NULL)
    {
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
    h->channel_count = (config->protocol == RCBUS_PROTOCOL_IBUS)
        ? RCBUS_IBUS_CHANNEL_COUNT
        : RCBUS_SBUS_CHANNEL_COUNT;
    h->digital_ch17         = 0U;
    h->digital_ch18         = 0U;
    h->sbus_frame_lost_flag = 0U;
    h->sbus_failsafe_flag   = 0U;
    h->frame_count          = 0U;
    h->error_count          = 0U;
    /* Пока не пришёл первый кадр, таймаут потери связи отсчитывается с
     * момента Init(), а не "с начала времён" (иначе IsFrameLost() был бы
     * ложно 0 сразу после старта, до реального приёма хоть одного кадра). */
    h->last_frame_tick = HAL_GetTick();
    h->used = 1U;

    HAL_StatusTypeDef rx_status = HAL_UARTEx_ReceiveToIdle_DMA(config->huart, h->rx_buffer, RCBUS_RX_BUFFER_LEN);
    if ((rx_status != HAL_OK) && (rx_status != HAL_BUSY))
    {
        h->used = 0U; /* откатываем выделение слота - приём не запустился */
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

        uint8_t ok = (h->config.protocol == RCBUS_PROTOCOL_IBUS)
            ? RCBUS_ParseIBusFrame(h, h->rx_buffer, Size)
            : RCBUS_ParseSBusFrame(h, h->rx_buffer, Size);

        if (ok != 0U)
        {
            h->frame_count++;
            h->last_frame_tick = HAL_GetTick();
        }
        else
        {
            h->error_count++;
        }

        RCBUS_RestartReception(h);
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
        /* DMA-приём после ошибки USART (overrun/framing/noise) сам себя не
         * восстанавливает - обязательно останавливаем и перезапускаем, иначе
         * приём каналов "зависает" молча после первой же помехи на линии. */
        (void)HAL_UART_AbortReceive(huart);
        RCBUS_RestartReception(h);
    }
}
