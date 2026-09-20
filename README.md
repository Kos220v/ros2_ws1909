# ros2_ws1909 — гусеничный робот, GPS + Nav2

ROS 2 **Jazzy / Ubuntu 24.04**. Драйверы VESC, BNO086 IMU (Qwiic/I²C), GNSS, ELRS и YDLIDAR;
одометрия по тахометрам + BNO086 и глобальная GPS-локализация, последовательный маршрут WGS84 и лидарный объезд
препятствий через Nav2.

**[Подробная настройка Nav2, запуск и проверки безопасности →](docs/GPS_NAVIGATION_RU.md)**

**[Точная модель одометрии: счётчики, IMU, синхронизация и калибровка →](docs/ODOMETRY_RU.md)**

**[Подключение BNO086 к Raspberry Pi 5 по I²C →](docs/BNO086_RASPBERRY_PI5_RU.md)**

**BNO086 требует шесть проводов:** I²C/питание плюс RST → GPIO17 (пин 11) и
INT → GPIO27 (пин 13), если эти линии свободны. Нужны `python3-libgpiod` и права
на GPIO RP1. После аппаратного сбоя IMU — явный перезапуск стека на стоянке,
не автоматический respawn с продолжением старой миссии.

Основные команды (после установки зависимостей и `colcon build`):

```bash
# Разные терминалы; сначала прочитайте инструкцию и настройте железо.
ros2 launch project_start start.launch.py declination_deg:=0.0  # заменить местным склонением
ros2 launch project_start localization.launch.py
ros2 launch project_start navigation.launch.py
ros2 run project_start gps_route --ros-args -p route_file:=$HOME/my_route.yaml
```

Движение требует исправных датчиков, явного режима **AUTO** и запущенной миссии.
Пример `src/project_start/config/route.example.yaml` отключён до замены координат.
Footprint, stop zone, параметры привода и IMU требуют натурной калибровки.
Обычный GPS и 2D-лидар не обеспечивают безопасное движение по дорогам общего
пользования; необходимы оператор, закрытая испытательная площадка и аппаратный E-stop.
В среде разработки выполнены unit/static-проверки, не испытания на реальном роботе.
