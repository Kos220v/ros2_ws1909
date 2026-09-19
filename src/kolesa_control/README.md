# kolesa_control — VESC hardware counters

Два FS75100/VESC по UART. Порты проекта Pi 5: левый `/dev/ttyAMA3`, правый
`/dev/ttyAMA4` (проверить overlays и физическое подключение).

## Разделение функций

- `kolesa_control`: duty-команды, проверка телеметрии, разворачивание переполнений
  аппаратных тахометров в Python integer, калибровка метров на тик.
- `counter_odometry` (пакет `project_start`): локальная XY/yaw-одометрия по
  **приращениям счётчиков и BNO086 на общей временной шкале**.
- `ekf_global`: глобальная локализация с GNSS. Только он публикует `map→odom`;
  только counter_odometry публикует `odom→base_link`.

Полная математика и настройка: [ODOMETRY_RU.md](../../docs/ODOMETRY_RU.md).

## Тахометр — не RPM и не абсолютная координата

`tachometer` — знаковый int32 аппаратного счётчика коммутационных тиков.
При реверсе знак приращения меняется. `tachometer_abs` — счётчик общего пробега
по обоим направлениям (на проводе тоже int32, арифметика разности по модулю 2³²).
Это **не** `abs(tachometer)` и не число механических оборотов без калибровки.
Для sensorless VESC качество счёта зависит и от оценки положения ротора самим
контроллером — наличие значения в телеметрии ещё не гарантирует точный энкодер.

```text
k_left/right = distance_per_revolution / tacho_counts_per_revolution
              × odometry_scale × left/right_odometry_scale
s_left/right = unwrapped_signed_ticks × k_left/right
travel_left/right = unwrapped_tachometer_abs_ticks × k_left/right
```

Путь не рассчитывается как `RPM × dt`, `cmd_vel × dt` или `speed × dt`.
Числа тиков суммируются **целочисленно** только для разворачивания аппаратного
счётчика при переполнениях. Метры каждый раз вычисляются из полного числа тиков.
`dt` применяется только к производной для скорости и проверке правдоподобия.

eRPM и duty используются в диагностике, но не для расчёта пути.
`JointState.position` — механический угол, без эмпирической поправки скольжения
`odometry_scale`. Разность гусениц не используется как измеренный yaw; в локальной
одометрии её отличие от yaw IMU служит только индикатором неопределённости.

## Топики

| Топик | Тип / смысл |
|---|---|
| `/cmd_vel` | Twist, вход от mux |
| `/kolesa/track_left`, `/kolesa/track_right` | `tracked_robot_interfaces/TrackTicks`, независимые timestamps каждого борта, счётчики, коэффициент м/тик, UUID сессии, valid |
| `/odom/vesc` | Odometry: только vx, pose не измерена (covariance 1e6); совместимость/guard |
| `/kolesa/distance_left`, `/kolesa/distance_right`, `/kolesa/distance_center` | Float64, знаковое перемещение в метрах; center — полусумма бортов, **не X карты** |
| `/joint_states` | JointState, механический угол и скорость гусениц |
| `/kolesa/diagnostics` | сырые счётчики, signed_counts, total_abs_counts, distance_m, travel_m, fault/возраст/валидность |

Для вычисления одометрии использовать **TrackTicks с timestamp**, а не Float64
без timestamp. Каждый борт публикуется независимо, без ложного предположения,
что два UART ответили одновременно. UUID меняется при каждом старте ноды;
counter_odometry запрещает склеивать отсчёты разных сессий.

## Сборка/запуск

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws1909
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-up-to project_start
source install/setup.bash
# Общий hardware launch уже запускает kolesa_control; не запускать две копии!
ros2 launch project_start start.launch.py
# В другом терминале:
ros2 launch project_start localization.launch.py
```

Для изолированного стенда: `colcon build --packages-up-to kolesa_control`,
затем `ros2 launch kolesa_control kolesa_control.launch.py`.
Новый CMake-пакет `tracked_robot_interfaces` должен собраться **до** Python-ноды;
`--packages-select kolesa_control` без ранее собранного интерфейса недостаточно.

## Параметры

Все параметры read-only при работе: изменить конфиг и перезапустить **на стоянке**.
Не сбрасывать масштабы/счётчики посреди движения.

| Параметр | Значение по умолчанию ноды | Назначение |
|---|---:|---|
| left_port / right_port | AMA3 / AMA4 | UART |
| baud | 115200 | скорость UART |
| wheel_separation | 0.48 м | раскладка команд по бортам, не источник курса |
| tacho_counts_per_revolution | 2157.0 | измеренные тики на оборот выходного вала; перепроверить |
| distance_per_revolution | 2.011 м | эффективное продвижение за оборот; перепроверить |
| odometry_scale | 1.0 | общий коэффициент; общий hardware launch задаёт прежний 1.15 |
| left_odometry_scale / right_odometry_scale | 1.0 / 1.0 | индивидуальная поправка бортов |
| encoder_invert_left / right | false / true | знак тахометра; оба пути растут при движении вперёд |
| invert_left / right | false / false | знак команды; в launch правый инвертирован |
| invert_angular | false | знак поворота команды |
| max_linear_velocity / max_angular_velocity | 1.0 / 1.0 | ограничения м/с и рад/с |
| duty_min / duty_max | .03 / 1.0 | duty; launch ограничивает duty_max до .6 |
| control_rate / telemetry_rate | 50 / 20 Гц | управление / запросы телеметрии |
| cmd_timeout | .5 с | остановка при отсутствии команды |
| telemetry_stale_timeout | .5 с | отсутствие свежей телеметрии запрещает команды обоим бортам |
| telemetry_pair_max_skew | .10 с | максимальная разница времён для legacy vx/скалярных публикаций |
| tacho_jump_margin | 3.0 | запас к физически достижимому приращению |
| min_tacho_jump_threshold | 10 тиков | нижний порог допуска квантования |
| publish_odom / publish_joint_states / publish_diagnostics | true | совместимость/визуализация/диагностика |

Предел скачка учитывает `v_max + w_max × wheel_separation/2` и масштаб м/тик.
Подозрительный скачок или уменьшение общего счётчика пробега защёлкивает fault:
принятые ранее путь/угол **не меняются**, скорость недействительна. Требуются
остановка, проверка VESC и перезапуск узлов локализации. Малый сброс около нуля,
неотличимый от физического движения по этим двум счётчикам, распознать абсолютно
надёжно нельзя — обычный GET_VALUES не содержит надёжного boot ID контроллера.

После длинной паузы путь восстанавливается по счётчикам, если они непрерывны;
первый интервал скорости недействителен. Но **XY через паузу IMU не восстанавливается**:
counter_odometry блокируется, потому что повороты в пропущенном интервале неизвестны.

## Калибровка и безопасность

Измерить продвижение рулеткой/RTK на прямой 10–20 м. Для каждого борта:
`k_measured = D / abs(delta_signed_counts)`; поправка индивидуального scale =
старый scale × `D / measured_distance`. Повторить вперёд/назад несколько раз,
с учётом покрытия/нагрузки. Старые параметры `wheel_radius`, `pole_pairs`,
`gear_ratio` и автоматическое вычисление тиков не поддерживаются.

Duty — разомкнутое управление, **не** точный регулятор скорости. Команда 0.2 м/с
не гарантирует 0.2 м/с в реальности. При fault/stale хотя бы одного борта команды
обоим обнуляются; отсутствующий ответ с нулевой скоростью не объявляется здоровым.
Неполные UART-ответы отбрасываются целиком; повторные ответы не получают новую
метку времени. Штатный timeout VESC и аппаратный E-stop обязательны: нулевой duty
не равнозначен гарантированному торможению на уклоне.
