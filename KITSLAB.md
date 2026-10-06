# Black Magic Debug для Лобзика (KitsLab)

Форк [Black Magic Debug](https://codeberg.org/blackmagic-debug/blackmagic) под отладчик
**Лобзик** от KitsLab — зонд на STM32L432KC, работает как Black Magic Probe (USB 1d50:6018,
GDB-сервер и UART прямо в зонде). Здесь — чем этот репозиторий отличается от оригинала.

Основа — `main` с Codeberg (официальный репозиторий; GitHub-зеркало `blackmagic-debug`
остановилось в феврале 2026), последнее слияние — `082456b3` от 27.09.2026. Наши правки
лежат сверху отдельными коммитами, апстрим вливается через merge.

## Что добавлено

### Платформа `l432` — сам Лобзик

`src/platforms/l432/`, `cross-file/l432.ini`. В апстриме такой платформы нет.

- **MCU:** STM32L432KC, 80 МГц от HSI16 + PLL, USB FS без кварца.
- **Интерфейсы цели:** JTAG и SWD через трансляторы уровней, NRST, UART цели
  (USART2), SWO в режиме UART (USART1 RX на PB7, он же TDO).
- **Без загрузчика BMD.** Образ собирается с адреса `0x08000000` (`bmd_bootloader = false`)
  и шьётся по SWD или через встроенный DFU-загрузчик ST. `DFU_DETACH` от `dfu-util`
  перезапускает Лобзик в системный загрузчик ST, так что прошивку можно обновлять
  без кнопки. Кнопка SW1 (PH3/BOOT0) нужна только для первой прошивки или после неудачной.
- **Перевод SWDIO на приём.** У транслятора уровней канал TDI делит линию направления
  со SWDIO. Когда SWDIO переключается на приём, TDI тоже переводится во вход,
  чтобы зонд и транслятор не боролись за линию.
- **Мягкий старт питания цели.** `mon tpwr enable` открывает ключ питания короткими
  импульсами. Между импульсами прошивка через VREFINT следит за просадкой своей шины
  3.3 В. Так Лобзик заряжает ёмкости цели (сотни мкФ) и сам не перезагружается.
  Пока питание нарастает, NRST цели прижат к земле. Если за 500 мс на цели нет 3.0 В,
  ключ закрывается (КЗ или слишком тяжёлая нагрузка).
- **Напряжение цели** считается от реального VDDA через VREFINT, а не от
  номинальных 3.3 В.

Выводы — в [src/platforms/l432/README.md](src/platforms/l432/README.md).

### Цель НИИЭТ К1921ВГ015 (RISC-V)

`src/target/niiet.c`, группа целей `niiet`. В апстриме нет.

- Распознаётся по JTAG IDCODE `0x00000d5b` (RISC-V DTM, debug v0.13), `mvendorid` 0x0d2d
  и регистру `PMUSYS_CHIPID`.
- Встроенная флеш 1 МБ с адреса `0x80000000` (страница 4 КБ): стирание, запись,
  `mon erase_mass`, `load` из GDB.
- Команды `mon swreset` и `mon hwreset`; сообщает, если чип в сервисном режиме.
- Драйвер написан по мотивам драйвера OpenOCD от НИИЭТ.

Это цель для отладочной платы «Вагонка».

## Что изменено в общем коде

Правки минимальные: только то, без чего не собрать платформу на STM32L4.

- `aux_serial.c/.h`, `usb_serial.c`, `swo_uart.c`, `swo_manchester.c`, `serialno.c` —
  STM32L4 добавлен в те же `#if`, где перечислены остальные семейства STM32 (для L4
  DMA-буфер UART 64 байта, как у F0). В `swo_uart.c` для DMA с `CSELR` выбирается
  запрос канала.
- `src/platforms/common/stm32/meson.build` — зависимость `platform_stm32l4` (Cortex-M4F).
- `meson_options.txt` — пробник `l432` и группа целей `niiet`.
- `jtag_devs.c`, `jep106.h`, `riscv32.c` — распознавание К1921ВГ015.

Всё остальное — как в апстриме, в том числе `jtagtap.c`/`swdptap.c` и проверка BYPASS
в `jtag_scan.c`.

## Сборка

```sh
meson setup build --cross-file cross-file/l432.ini
meson compile -C build
```

Результат: `build/blackmagic_l432_firmware.bin` и `.elf`. Набор целей задан в
`cross-file/l432.ini`. Для отладочного вывода (`mon debug_bmp`) нужен
`-Ddebug_output=true`.

Прошивка по DFU:

```sh
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/blackmagic_l432_firmware.bin
```

Версия прошивки берётся из `git describe` по тегам `lobzik-*` (видна в `mon version`).

## Обновление с апстрима

```sh
git fetch upstream            # codeberg.org/blackmagic-debug/blackmagic
git merge upstream/main       # merge, не rebase
```

Конфликты обычно бывают в общих `#if` по семействам STM32: апстрим добавляет туда
свои семейства, мы — `STM32L4`, оставлять нужно оба. `deps/libopencm3` тоже берётся
с Codeberg. Если в `deps/libopencm3.wrap` сменилась ревизия, обновите подмодуль вручную,
иначе meson не найдёт `opencm3_stm32l4`.

## Лицензия

Как у оригинала: GPL-3.0-or-later (часть файлов — BSD/MIT, см. `COPYING*`).
