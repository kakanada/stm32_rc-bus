# rc_bus — справочник по API

Дисклеймер: при расхождениях с `rc_bus.h` ориентируйтесь на `.h`, он первичен
(этот справочник — сжатый пересказ Doxygen-комментариев).

## Оглавление

- [Выбор протокола и настройка UART](#выбор-протокола-и-настройка-uart)
- [Типы](#типы)
- [Регистрация экземпляра](#регистрация-экземпляра)
- [Чтение каналов](#чтение-каналов)
- [Обработчики HAL-колбэков](#обработчики-hal-колбэков)
- [Ограничения](#ограничения)

---

## Выбор протокола и настройка UART

Один физический UART = один экземпляр библиотеки = один протокол (i-BUS либо
S.BUS), задаётся в `RCBUS_Config_t.protocol`. Разные экземпляры (разные UART)
могут одновременно работать в разных протоколах.

| Протокол | Baud   | Биты данных   | Чётность | Стоп-биты | Сигнал         |
|----------|--------|---------------|----------|-----------|----------------|
| i-BUS    | 115200 | 8             | нет      | 1         | прямой         |
| S.BUS    | 100000 | 9 (8+чётн.)   | чётная   | 2         | инвертированный|

`RCBUS_Init()` проверяет соответствие `huart->Init` этой таблице и вернёт
`NULL`, если настройки не совпадают с выбранным `protocol` — конфигурация
UART делается в CubeMX самим пользователем, библиотека периферию не
переинициализирует, только проверяет и использует.

Инверсия сигнала S.BUS — только аппаратная: на STM32H7 включается через
CubeMX (UART Advanced Init → Rx Pin Active Level Inversion), на STM32F4
аппаратной инверсии в USART нет — обязателен внешний инвертор сигнала между
приёмником и ногой RX МК.

DMA Rx должен быть настроен в режиме **Normal** (не Circular).

---

## Типы

### `RCBUS_Protocol_t`

| Значение             | Смысл                                          |
|----------------------|-------------------------------------------------|
| `RCBUS_PROTOCOL_IBUS` | FlySky i-BUS (напр. FS-iA6B/FS-X6B)             |
| `RCBUS_PROTOCOL_SBUS` | Futaba S.BUS (и большинство совместимых)        |

### `RCBUS_Config_t`

| Поле | Тип | Описание |
|---|---|---|
| `huart` | `UART_HandleTypeDef*` | UART экземпляра, уже проинициализирован CubeMX-кодом под нужный протокол |
| `protocol` | `RCBUS_Protocol_t` | i-BUS или S.BUS |
| `failsafe_timeout_ms` | `uint32_t` | таймаут "потери связи", мс; `0` = использовать значение по умолчанию (100 мс) |

### `RCBUS_Handle_t`

Возвращается `RCBUS_Init()` по указателю. Память статическая (пул на
`RCBUS_MAX_INSTANCES` элементов), живёт всё время работы программы.

Публичные поля (можно читать): `config` (копия переданной конфигурации),
`channels[]` (текущие значения каналов, ~988..2012, центр 1500),
`channel_count` (14 для i-BUS, 16 для S.BUS), `digital_ch17`/`digital_ch18`
(дискретные каналы, только у S.BUS), `frame_count`/`error_count` (счётчики
для диагностики качества линии), `index` (позиция в пуле). Остальные поля —
внутренние, не трогать напрямую.

---

## Регистрация экземпляра

### `RCBUS_Handle_t *RCBUS_Init(const RCBUS_Config_t *config)`

Регистрирует экземпляр на указанном `huart`: проверяет соответствие
`huart->Init` выбранному протоколу и запускает аппаратный приём
(`HAL_UARTEx_ReceiveToIdle_DMA` + отключение прерывания half-transfer DMA).
Повторный вызов с тем же `huart` идемпотентен — вернёт указатель на тот же
хэндл, применив поверх него новую конфигурацию.

Возвращает указатель на хэндл, либо `NULL` при ошибке: `config`/`huart`
некорректны, `huart->Init` не соответствует `protocol`, исчерпан
`RCBUS_MAX_INSTANCES`, либо не удалось запустить приём.

---

## Чтение каналов

### `uint16_t RCBUS_GetChannel(const RCBUS_Handle_t *h, uint8_t channel_index)`

Значение одного канала (~988..2012, центр 1500). `0`, если `h == NULL` или
`channel_index >= h->channel_count`.

### `HAL_StatusTypeDef RCBUS_GetChannels(const RCBUS_Handle_t *h, uint16_t *out, uint8_t count)`

Копирует `min(count, h->channel_count)` каналов в `out`. `HAL_OK`;
`HAL_ERROR`, если `h == NULL` или `out == NULL`.

### `uint8_t RCBUS_IsFrameLost(const RCBUS_Handle_t *h)`

`1`, если валидный кадр не приходил дольше эффективного
`failsafe_timeout_ms`, либо (для S.BUS) в последнем кадре был установлен бит
"frame lost". `1` также при `h == NULL`.

### `uint8_t RCBUS_IsFailsafe(const RCBUS_Handle_t *h)`

Для S.BUS — явный бит "failsafe activated" из кадра ИЛИ `RCBUS_IsFrameLost()`.
Для i-BUS протокол не содержит отдельного флага failsafe — функция
равносильна `RCBUS_IsFrameLost()`. `1` также при `h == NULL`.

---

## Обработчики HAL-колбэков

### `void RCBUS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)`

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
флагов — S.BUS) и заново запускает приём следующего кадра.

### `void RCBUS_UART_ErrorCallback(UART_HandleTypeDef *huart)`

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

## Ограничения

- Только приём каналов (RX) — без телеметрии и без передачи.
- Только классический 25-байтный кадр S.BUS, без S.BUS2-телеметрии в слотах.
- Требуется HAL с `HAL_UARTEx_ReceiveToIdle_DMA()`.
- Инверсия сигнала S.BUS — только аппаратная (см. раздел про настройку UART).
