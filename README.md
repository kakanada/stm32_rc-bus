# rc_bus — приём каналов и телеметрия РУ (i-BUS / S.BUS) для STM32

rc_bus — библиотека для STM32 F4/H7 из двух модулей: приём каналов
управления по FlySky i-BUS (пример приёмника — FS-iA6B) или Futaba S.BUS
(пример — приёмники RadioMaster с S.BUS-выходом), и ответ телеметрией на
опрос по отдельной шине датчиков i-BUS. Работает поверх стандартного HAL
UART/DMA, без bit-bang и без блокирующего опроса в основном цикле.

## Возможности

- Оба протокола каналов в одной библиотеке — `RCBUS_Config_t.protocol`
  выбирает i-BUS или S.BUS для конкретного экземпляра (UART-линии); разные
  линии могут одновременно работать в разных протоколах.
- Приём полностью аппаратный: DMA + определение простоя линии (IDLE) через
  `HAL_UARTEx_ReceiveToIdle_DMA()` — без опроса, без программных таймеров
  между байтами, без блокировок в основном цикле.
- Каналы нормализованы в единую шкалу "ширина импульса в мкс" (~988..2012,
  центр 1500) независимо от протокола — код приложения не меняется при
  переключении i-BUS ⇄ S.BUS.
- Контроль целостности: контрольная сумма кадра (i-BUS), явные биты
  frame-lost/failsafe (S.BUS) + таймаут по факту приёма валидного кадра —
  единая функция `RCBUS_IsFrameLost()`/`RCBUS_IsFailsafe()` для обоих.
  Корректно принимает кадры каналов и от приёмников в режиме S.BUS2 (см.
  `sbus2_telemetry_signal`).
- Восстановление после ошибок линии (framing/noise/overrun) — приём сам
  перезапускается, не "зависает" после первой же помехи.
- Телеметрия i-BUS: до `RCBUS_TELEMETRY_MAX_ADDRESS` (15) виртуальных
  датчиков на шину, неблокирующий приём опроса + неблокирующая передача
  ответа (`HAL_UART_Transmit_IT`) с корректным переключением направления на
  однопроводной half-duplex линии.
- Статические пулы экземпляров без malloc, идемпотентный `Init()`.
- Необязательная интеграция с **stm32_logger**: определите `RC_BUS_LOGGER_ENABLED`,
  чтобы ошибки инициализации, битые кадры и сбои UART отправлялись через
  `LOGGER_Log()`/`LOGGER_Mark()` (коды в `logger_codes.h`, `LOG_ADDR_RC_BUS`).
  Без этого define зависимости от `logger.h` нет вообще.

## Требования к настройке в CubeMX

Приём каналов (`rc_bus.h`):

| Протокол | Baud   | Данные        | Чётность | Стоп-биты | Сигнал        |
|----------|--------|---------------|----------|-----------|---------------|
| i-BUS    | 115200 | 8 бит         | нет      | 1         | прямой        |
| S.BUS    | 100000 | 9 бит (8+ч.)  | чётная   | 2         | **инверт.** ¹ |

¹ S.BUS физически инвертирован. На H7 включается в CubeMX (UART Advanced
Init → Rx Pin Active Level Inversion); на F4 аппаратной инверсии в USART нет —
нужен внешний инвертор (например, простой транзисторный ключ) между линией
приёмника и ногой RX МК. Без этого S.BUS приниматься не будет.

Для обоих протоколов обязательно: DMA Rx в режиме **Normal** (не Circular),
USART global interrupt включён.

Телеметрия (`rc_bus_telemetry.h`) — отдельная физическая линия (не та же, что
приём каналов): Mode → **Half-Duplex Selection** (Single Wire), 115200 8N1,
DMA Rx Normal, USART global interrupt включён.

## Быстрый старт

### Приём каналов

```c
#include "rc_bus.h"

static RCBUS_Handle_t *rc;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    RCBUS_UART_RxEventCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    RCBUS_UART_ErrorCallback(huart);
}

void AppInit(void)
{
    RCBUS_Config_t cfg = {
        .huart = &huart2,               /* уже проинициализирован CubeMX-ом под i-BUS/S.BUS */
        .protocol = RCBUS_PROTOCOL_IBUS, /* или RCBUS_PROTOCOL_SBUS */
        .failsafe_timeout_ms = 0,        /* 0 = значение по умолчанию, 100 мс */
    };
    rc = RCBUS_Init(&cfg);
}

void AppLoop(void)
{
    if (RCBUS_IsFailsafe(rc))
    {
        /* например, перевести приводы в безопасное положение */
        return;
    }
    uint16_t throttle = RCBUS_GetChannel(rc, 2); /* канал 3 (индекс с 0), ~988..2012 */
}
```

### Телеметрия (датчик на шине i-BUS)

```c
#include "rc_bus_telemetry.h"

static RCBUS_TelemetryHandle_t *tlm;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    RCBUS_TelemetryUART_RxEventCallback(huart, Size);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    RCBUS_TelemetryUART_TxCpltCallback(huart);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    RCBUS_TelemetryUART_ErrorCallback(huart);
}

void AppInit(void)
{
    RCBUS_TelemetryConfig_t cfg = { .huart = &huart3 }; /* Half-Duplex, 115200 8N1 */
    tlm = RCBUS_TelemetryInit(&cfg);
    RCBUS_TelemetryRegisterSensor(tlm, 1, RCBUS_IBUS_SENSOR_TYPE_VOLTAGE, 2);
}

void AppLoop(void)
{
    uint16_t battery_mv = 7400; /* например, из АЦП */
    RCBUS_TelemetrySetValue(tlm, 1, battery_mv);
}
```

## Честные ограничения

- Приём каналов (`rc_bus.h`) не разбирает телеметрию S.BUS2 (обмен в
  "слотах" между кадрами каналов) — у Futaba нет публичной спецификации
  этого обмена. Кадры каналов от приёмников в режиме S.BUS2 при этом
  принимаются корректно (см. `sbus2_telemetry_signal` — только детект факта
  сигнализации, не разбор данных).
- Телеметрия (`rc_bus_telemetry.h`) реализована только для i-BUS — у S.BUS
  нет отдельной шины датчиков, аналогичной i-BUS SENSOR.
- Каталог кодов типов датчиков телеметрии i-BUS не встроен (кроме
  `RCBUS_IBUS_SENSOR_TYPE_VOLTAGE`) — код типа передаётся сырым байтом,
  сверьтесь с таблицей вашего приложения передатчика.
- Требует HAL с `HAL_UARTEx_ReceiveToIdle_DMA()` — актуальные пакеты CubeF4 /
  CubeH7 его содержат; в очень старых пакетах HAL потребуется обновление.
- Программная инверсия сигнала S.BUS невозможна в принципе — периферия UART
  ищет старт-бит уже на инвертированном сигнале, поэтому инверсия только
  аппаратная (см. таблицу выше).

Полный список функций и типов — см. [API_REFERENCE.md](API_REFERENCE.md).

## Лицензия

Библиотека распространяется на условиях **PolyForm Noncommercial License 1.0.0** — свободное
использование, копирование, изменение и распространение для любых НЕКОММЕРЧЕСКИХ целей, при
условии сохранения уведомления об авторских правах. Коммерческое использование требует отдельного
разрешения правообладателя. Полный текст — файл [LICENSE](LICENSE).
