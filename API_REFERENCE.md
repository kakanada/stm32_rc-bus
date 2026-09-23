# rc_bus — справочник по API

Дисклеймер: при расхождениях с `.h`-файлами ориентируйтесь на них, они
первичны (этот справочник — сжатый пересказ Doxygen-комментариев).

## Оглавление

- [Модуль приёма каналов (rc_bus.h)](#модуль-приёма-каналов-rc_bush)
  - [Выбор протокола и настройка UART](#выбор-протокола-и-настройка-uart)
  - [Типы](#типы)
  - [Регистрация экземпляра](#регистрация-экземпляра)
  - [Чтение каналов](#чтение-каналов)
  - [Обработчики HAL-колбэков](#обработчики-hal-колбэков)
- [Модуль телеметрии i-BUS (rc_bus_telemetry.h)](#модуль-телеметрии-i-bus-rc_bus_telemetryh)
  - [Настройка UART (Half-Duplex)](#настройка-uart-half-duplex)
  - [Типы (телеметрия)](#типы-телеметрия)
  - [Регистрация экземпляра и датчиков](#регистрация-экземпляра-и-датчиков)
  - [Обработчики HAL-колбэков (телеметрия)](#обработчики-hal-колбэков-телеметрия)
  - [Протокол опроса — детали](#протокол-опроса--детали)
- [Общие ограничения](#общие-ограничения)

---

## Модуль приёма каналов (rc_bus.h)

### Выбор протокола и настройка UART

Один физический UART = один экземпляр библиотеки = один протокол (i-BUS,
S.BUS либо CRSF), задаётся в `RCBUS_Config_t.protocol`. Разные экземпляры
(разные UART) могут одновременно работать в разных протоколах.

| Протокол | Baud   | Биты данных   | Чётность | Стоп-биты | Сигнал         |
|----------|--------|---------------|----------|-----------|----------------|
| i-BUS    | 115200 | 8             | нет      | 1         | прямой         |
| S.BUS    | 100000 | 9 (8+чётн.)   | чётная   | 2         | инвертированный|
| CRSF     | 420000 | 8             | нет      | 1         | прямой         |

`RCBUS_Init()` проверяет соответствие `huart->Init` этой таблице и вернёт
`NULL`, если настройки не совпадают с выбранным `protocol` — конфигурация
UART делается в CubeMX самим пользователем, библиотека периферию не
переинициализирует, только проверяет и использует.

Инверсия сигнала S.BUS — только аппаратная: на STM32H7 включается через
CubeMX (UART Advanced Init → Rx Pin Active Level Inversion), на STM32F4
аппаратной инверсии в USART нет — обязателен внешний инвертор сигнала между
приёмником и ногой RX МК. Если приёмник поддерживает CRSF — на F4 это
единственный из трёх протоколов, не требующий инвертора (сигнал прямой).

DMA Rx должен быть настроен в режиме **Normal** (не Circular). Mode UART —
**Asynchronous** либо **Half-Duplex Selection** (Single Wire), на выбор, для
любого из трёх протоколов: модуль только принимает, приём через
`HAL_UARTEx_ReceiveToIdle_DMA()` не зависит от того, какой из двух режимов
настроен в CubeMX. `RCBUS_Init()` проверяет только параметры линии из
таблицы выше, сам Mode не проверяется.

Кадры каналов от приёмников в режиме **S.BUS2** принимаются штатно (см. ниже
`sbus2_telemetry_signal`) — библиотека допускает любое из известных значений
байта конца кадра (`0x00`/`0x04`/`0x14`/`0x24`/`0x34`), а не только
классическое `0x00`.

На линии CRSF приёмник, помимо кадра каналов (`RC_CHANNELS_PACKED`, тип
`0x16`), может присылать и другие типы кадров (например, `LINK_STATISTICS`) —
`RCBUS_UART_RxEventCallback()` распознаёт их по CRC и корректно пропускает,
не считая ошибкой линии и не обновляя `channels[]`/`frame_count`.

### Типы

#### `RCBUS_Protocol_t`

| Значение             | Смысл                                          |
|----------------------|-------------------------------------------------|
| `RCBUS_PROTOCOL_IBUS` | FlySky i-BUS (напр. FS-iA6B/FS-X6B)             |
| `RCBUS_PROTOCOL_SBUS` | Futaba S.BUS (и большинство совместимых)        |
| `RCBUS_PROTOCOL_CRSF` | TBS Crossfire / ExpressLRS (CRSF)               |

#### `RCBUS_Config_t`

| Поле | Тип | Описание |
|---|---|---|
| `huart` | `UART_HandleTypeDef*` | UART экземпляра, уже проинициализирован CubeMX-кодом под нужный протокол |
| `protocol` | `RCBUS_Protocol_t` | i-BUS, S.BUS или CRSF |
| `failsafe_timeout_ms` | `uint32_t` | таймаут "потери связи", мс; `0` = использовать значение по умолчанию (100 мс) |

#### `RCBUS_Handle_t`

Возвращается `RCBUS_Init()` по указателю. Память статическая (пул на
`RCBUS_MAX_INSTANCES` элементов), живёт всё время работы программы.

Публичные поля (можно читать): `config` (копия переданной конфигурации),
`channels[]` (текущие значения каналов, ~988..2012, центр 1500),
`channel_count` (14 для i-BUS, 16 для S.BUS и CRSF), `digital_ch17`/
`digital_ch18` (дискретные каналы, только у S.BUS), `sbus2_telemetry_signal`
(приёмник сигнализирует S.BUS2-телеметрию в кадре — только детект факта, см.
[Общие ограничения](#общие-ограничения)), `frame_count`/`error_count`
(счётчики для диагностики качества линии), `index` (позиция в пуле).
Остальные поля — внутренние, не трогать напрямую.

### Регистрация экземпляра

#### `RCBUS_Handle_t *RCBUS_Init(const RCBUS_Config_t *config)`

Регистрирует экземпляр на указанном `huart`: проверяет соответствие
`huart->Init` выбранному протоколу и запускает аппаратный приём
(`HAL_UARTEx_ReceiveToIdle_DMA` + отключение прерывания half-transfer DMA).
Повторный вызов с тем же `huart` идемпотентен — вернёт указатель на тот же
хэндл, применив поверх него новую конфигурацию.

Возвращает указатель на хэндл, либо `NULL` при ошибке: `config`/`huart`
некорректны, `huart->Init` не соответствует `protocol`, исчерпан
`RCBUS_MAX_INSTANCES`, либо не удалось запустить приём.

### Чтение каналов

#### `uint16_t RCBUS_GetChannel(const RCBUS_Handle_t *h, uint8_t channel_index)`

Значение одного канала (~988..2012, центр 1500). `0`, если `h == NULL` или
`channel_index >= h->channel_count`.

#### `HAL_StatusTypeDef RCBUS_GetChannels(const RCBUS_Handle_t *h, uint16_t *out, uint8_t count)`

Копирует `min(count, h->channel_count)` каналов в `out`. `HAL_OK`;
`HAL_ERROR`, если `h == NULL` или `out == NULL`.

#### `uint8_t RCBUS_IsFrameLost(const RCBUS_Handle_t *h)`

`1`, если валидный кадр не приходил дольше эффективного
`failsafe_timeout_ms`, либо (для S.BUS) в последнем кадре был установлен бит
"frame lost". `1` также при `h == NULL`.

#### `uint8_t RCBUS_IsFailsafe(const RCBUS_Handle_t *h)`

Для S.BUS — явный бит "failsafe activated" из кадра ИЛИ `RCBUS_IsFrameLost()`.
Для i-BUS и CRSF протокол не содержит отдельного флага failsafe в кадре
каналов — функция равносильна `RCBUS_IsFrameLost()`. `1` также при
`h == NULL`.

### Обработчики HAL-колбэков

#### `void RCBUS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)`

Вызывать из своего `HAL_UARTEx_RxEventCallback()`:

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    RCBUS_UART_RxEventCallback(huart, Size);
}
```

Сама проверяет по полю `config.huart`, относится ли событие к
зарегистрированным здесь экземплярам; разбирает кадр под протокол этого
экземпляра (контрольная сумма — i-BUS, распаковка 11-битных каналов и байт
флагов — S.BUS, CRC8 и тип кадра — CRSF) и заново запускает приём следующего
кадра. На CRSF кадры протокола, отличные от `RC_CHANNELS_PACKED` (например,
`LINK_STATISTICS`), распознаются и молча пропускаются — не в счётчик ошибок.

#### `void RCBUS_UART_ErrorCallback(UART_HandleTypeDef *huart)`

Вызывать из своего `HAL_UART_ErrorCallback()`:

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    RCBUS_UART_ErrorCallback(huart);
}
```

Обязательна к подключению: без неё DMA-приём после ошибки линии
(framing/noise/overrun) не восстанавливается сам и приём каналов "зависает"
после первой же помехи.

---

## Модуль телеметрии i-BUS (rc_bus_telemetry.h)

Отдельная физическая half-duplex шина "SENSOR" приёмника i-BUS — МК отвечает
на опрос как один или несколько виртуальных датчиков. Это НЕ та же линия, на
которой `rc_bus.h` принимает каналы.

### Настройка UART (Half-Duplex)

CubeMX: Mode → **Half-Duplex Selection** (в коде это вызов
`HAL_HalfDuplex_Init()` вместо `HAL_UART_Init()`), 115200 8N1 (как обычный
i-BUS), DMA Rx **Normal**, USART global interrupt включён.

`RCBUS_TelemetryInit()` проверяет ОБА условия — параметры линии (115200 8N1)
И то, что аппаратный бит Half-Duplex (`CR3.HDSEL`) реально установлен, — и
вернёт `NULL`, если что-то не так (типичная ошибка — забыли выбрать
Half-Duplex Selection в CubeMX и получили обычный `HAL_UART_Init()`).

### Типы (телеметрия)

#### `RCBUS_TelemetryConfig_t`

| Поле | Тип | Описание |
|---|---|---|
| `huart` | `UART_HandleTypeDef*` | UART в режиме Half-Duplex, 115200 8N1 |

#### `RCBUS_TelemetrySensor_t`

| Поле | Тип | Описание |
|---|---|---|
| `registered` | `uint8_t` | 1, если адрес занят зарегистрированным датчиком |
| `type` | `uint8_t` | код типа датчика (сырой байт, см. `RCBUS_IBUS_SENSOR_TYPE_VOLTAGE`) |
| `length` | `uint8_t` | размер значения при передаче: 2 или 4 байта |
| `value` | `int32_t` | текущее значение, устанавливается приложением |

#### `RCBUS_TelemetryHandle_t`

Возвращается `RCBUS_TelemetryInit()` по указателю. Память статическая (пул на
`RCBUS_TELEMETRY_MAX_INSTANCES` элементов). Публичные поля: `config`,
`sensors[1..RCBUS_TELEMETRY_MAX_ADDRESS]`, `poll_count`/`error_count`
(диагностика), `index`.

### Регистрация экземпляра и датчиков

#### `RCBUS_TelemetryHandle_t *RCBUS_TelemetryInit(const RCBUS_TelemetryConfig_t *config)`

Проверяет настройки `huart` (см. выше) и запускает приём опроса. Повторный
вызов с тем же `huart` идемпотентен, но **сбрасывает регистрации всех
датчиков** этого экземпляра — регистрируйте их заново после повторного
`Init()`.

#### `HAL_StatusTypeDef RCBUS_TelemetryRegisterSensor(RCBUS_TelemetryHandle_t *h, uint8_t address, uint8_t type, uint8_t length)`

Регистрирует виртуальный датчик по адресу `1..15`. `length` — 2 или 4 байта
(иначе `HAL_ERROR`).

#### `HAL_StatusTypeDef RCBUS_TelemetrySetValue(RCBUS_TelemetryHandle_t *h, uint8_t address, int32_t value)`

Обновляет текущее значение — будет отдано при следующем опросе этого адреса.
`HAL_ERROR`, если датчик с таким адресом не зарегистрирован.

### Обработчики HAL-колбэков (телеметрия)

Три колбэка (в отличие от `rc_bus.h` — здесь ещё и передача ответа):

```c
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
```

`RCBUS_TelemetryUART_TxCpltCallback` обязателен: без него шина "зависнет" в
режиме передачи после первого же ответа (линия однопроводная, приём и
передача не могут идти одновременно).

### Протокол опроса — детали

Приёмник ("мастер") шлёт короткий 4-байтный кадр `[0x04, cmd|addr, chkLo, chkHi]`
(контрольная сумма — та же формула, что у сервo-кадра каналов i-BUS: `0xFFFF`
минус сумма предыдущих байт). `addr` — 4 младших бита (`1..15`), `cmd` — 4
старших:

| `cmd` | Значение | Ответ (если адрес наш) |
|---|---|---|
| `0x80` | DISCOVER — "датчик здесь?" | эхо того же 4-байтного кадра |
| `0x90` | TYPE — "какой тип?" | `[0x06, cmd|addr, type, length, chkLo, chkHi]` |
| `0xA0` | MEASUREMENT — "текущее значение?" | `[4+length, cmd|addr, value×length байт LE, chkLo, chkHi]` |

Опрос чужого адреса или неизвестная команда — не ошибка, ответа просто нет
(нормальное поведение на общей шине с несколькими датчиками).

---

## Общие ограничения

- Приём каналов (`rc_bus.h`) не содержит разбора телеметрии S.BUS2 (обмен в
  "слотах" между кадрами каналов) — у Futaba нет публичной спецификации этого
  обмена. `sbus2_telemetry_signal` — только детект факта сигнализации, не
  разбор данных.
- Телеметрия (`rc_bus_telemetry.h`) реализована только для i-BUS (отдельная
  шина датчиков) — у S.BUS такой отдельной шины в принципе нет.
- Каталог кодов типов датчиков i-BUS не встроен, кроме
  `RCBUS_IBUS_SENSOR_TYPE_VOLTAGE` — остальные передавайте сырым байтом,
  сверяясь с таблицей вашего приложения передатчика.
- Требуется HAL с `HAL_UARTEx_ReceiveToIdle_DMA()`.
- Инверсия сигнала S.BUS — только аппаратная (см. раздел про настройку UART).
- CRSF реализован только на приём (`RC_CHANNELS_PACKED`) — обратная
  телеметрия МК → приёмник по CRSF не реализована.
- Интеграция с **stm32_logger** — целиком опциональна и не тянется по
  умолчанию. Определите `RC_BUS_LOGGER_ENABLED` до включения `rc_bus.h`/
  `rc_bus_telemetry.h`, чтобы подключить `logger.h`/`logger_codes.h` и
  отправлять в `LOGGER_Log()`/`LOGGER_Mark()` события ошибок инициализации,
  ошибок UART и битых кадров (коды `LOG_CODE_RC_BUS_*`, адресное пространство
  `LOG_ADDR_RC_BUS` согласовано с проектом stm32_logger). Без define — ни
  одного упоминания `logger.h` в собранном коде.
