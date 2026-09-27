# bno08x_imu — ROS 2 Jazzy / BNO085 по I²C

Linux I²C (`smbus2.I2C_RDWR`), адрес конфигурации **0x4B** (не подтверждён для новой платы; сверить с Arduino), 3.3 В.
Подключение Raspberry Pi 5: 3V3 → физический pin 1, GND → pin 6,
SDA → GPIO2/pin 3, SCL → GPIO3/pin 5, **RST → GPIO17/pin 11,
INT → GPIO27/pin 13** (только если GPIO свободны). Не подключать к 5 В.

**Полная инструкция в репозитории: [`docs/BNO085_RASPBERRY_PI5_RU.md`](../../docs/BNO085_RASPBERRY_PI5_RU.md).**

```bash
sudo apt install python3-smbus2 python3-libgpiod gpiod i2c-tools
# Сначала включить I²C и настроить права по полной инструкции.
colcon build --symlink-install --packages-select bno08x_imu
source install/setup.bash
ros2 launch bno08x_imu imu.launch.py i2c_bus:=1 i2c_address:=75 declination_deg:=0.0
# Склонение 0.0 заменить местным; не запускать одновременно с project_start!
```

Топики: `/imu/data` (`Imu`), `/imu/mag` (`MagneticField`), `/imu/accuracy`
(`UInt8`, статус Rotation Vector), `/imu/azimuth` (`Float32`, истинный азимут
оси X датчика), `/diagnostics`. SensorData QoS (Best Effort) для измерений.
TF не публикуется — монтаж `base_link→imu_link` задаётся robot_state_publisher.

Реализован ограниченный SH-2/SHTP профиль: аппаратный RST, INT readiness, Product ID handshake,
Set Feature; отчёты 0x01 accel (с гравитацией), 0x02 calibrated gyro,
0x03 calibrated magnetic field, 0x05 magnetic Rotation Vector. Нет UART, SPI,
Game Rotation Vector, AR/VR stabilization, FRS/DCD-записи,
firmware update и полной сборки фрагментированных SHTP cargos. Неподдерживаемые
sensor-пакеты отвергаются, а не интерпретируются как случайные измерения.

Параметры только при запуске (read-only). 25 Гц основной набор / 10 Гц магнитометр
по умолчанию. Если очередь/протокол/шина неисправны, процесс прекращает публикацию,
защёлкивает ERROR в diagnostics; автоматического reset/respawn нет.
После устранения причины нужен перезапуск hardware/localization/navigation на стоянке. Недостаточная точность
ориентации не вызывает reset — даёт датчику закончить калибровку. Повторные
sequence numbers не обновляют свежесть, каждый Imu требует новых трёх отчётов.
Используется cooperative flock на I²C-адаптере: не запускать вторую копию.

Quaternion SH-2 `(i,j,k,real)` считается sensor→magnetic ENU. Истинная ENU:
`q_true = Rz(-declination_deg) * q_mag`, склонение восточное положительное.
Гироскоп/ускорение/магнитное поле остаются в **осях датчика**. Монтаж — только TF,
не дополнительная поправка yaw. `navsat_transform` должен получать уже
исправленный quaternion с нулевой собственной поправкой склонения.

Источники wire-формата, сверенные при реализации (не включены в пакет):
- [CEVA SH-2 sensor decoding](https://github.com/ceva-dsp/sh2/blob/master/sh2_SensorValue.c)
- [SparkFun BNO08x I²C HAL](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library)
- [CEVA BNO08x datasheet](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)

Тесты используют искусственные пакеты и fake-I²C, не физическую плату.
Реальные I²C timing, оси, магнитную калибровку и реакции на отказы необходимо
проверить на Pi 5 до включения автономии. Timestamps — оценка времени получения
на хосте с вычетом report delay, не синхронизация тактов SH-2 с UTC.

GPIO: libgpiod 1.x/2.x, `gpio_chip:=auto` ищет `pinctrl-rp1`; `rst_gpio:=17`,
`int_gpio:=27` — смещения линий RP1 (BCM). Не фиксировать номер gpiochip4.
GPIO запрашиваются эксклюзивно; INT опрашивается по уровню, не только по фронту.
При отсутствии прав или конфликте линий драйвер не запускается. Права группы
`gpio` и проверка UART overlays описаны в полной инструкции.

## Отдельная диагностика запуска (без ROS)

При `INT remained HIGH` не отключать защиту рабочего драйвера. Использовать
стендовую утилиту с отдельным сбросом для 0x4A/0x4B, наблюдением INT сразу после
RST и raw SHTP/Product ID проверкой даже при INT HIGH:

```bash
# Из корня workspace, моторы отключены, остальные клиенты остановлены:
PYTHONPATH="$PWD/src/bno08x_imu" /usr/bin/python3 -u -m bno08x_imu.diagnose \
  --confirm-stationary --i2c-bus 1
```

[Подробности и сохранение лога](../../docs/BNO08X_DIAGNOSTIC_RU.md).
Утилита ничего не публикует; результат не разрешает автономное движение.


Пакет переименован из `bno086_imu` в `bno08x_imu`; рабочий датчик проекта —
**BNO085**. Используется общий протокол SH-2/SHTP, не эмуляция отсутствующей IMU.
100 кГц задаётся ядром Raspberry Pi: `dtparam=i2c_arm_baudrate=100000`,
а не `rate_hz` (это частота отчётов, 25 Гц). Шаблон: `config/pi5_i2c_100khz.txt`.
Product ID/версия прошивки выводятся при старте и в diagnostics; это не автоматическое
определение модели кремния. Начальный INT timeout по-прежнему запрещает запуск.
Адрес из рабочего Arduino-скетча нужно явно перенести: `i2c_address:=74` или `75`.
