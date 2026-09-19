# -*- coding: utf-8 -*-

"""VESC duty control and hardware-counter odometry.

Signed displacement: unwrapped tachometer * calibrated meters/count.
Total track travel: unwrapped tachometer_abs * calibrated meters/count.
No RPM/speed integration is used for these distances. /odom/vesc publishes
only measured vx; scalar track distance must NOT masquerade as Cartesian x.
counter_odometry estimates local XY directly from timestamped ticks + BNO086.
robot_localization provides GNSS-corrected map localization.
"""

import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64
from tracked_robot_interfaces.msg import TrackTicks

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .vesc_driver import VescDriver
from .tachometer import Tachometer


# Ковариация «это значение не измерено, не используйте его».
UNMEASURED_COV = 1.0e6


def velocity_to_duty_cycle(velocity, max_velocity, duty_min=0.03, duty_max=1.0):
    """
    Преобразует целевую скорость в скважность (duty cycle) для VESC.

    Направление уже учтено в знаке velocity; функция ничего не знает
    про invert_left/right.
    """
    if abs(velocity) < 0.001:
        return 0.0
    normalized = abs(velocity) / max_velocity
    normalized = min(1.0, max(0.0, normalized))
    duty_magnitude = duty_min + normalized * (duty_max - duty_min)
    return duty_magnitude if velocity >= 0 else -duty_magnitude


class KolesaControl(Node):
    def __init__(self):
        super().__init__("kolesa_control")

        # ----------------------------------------------------------- параметры
        def p(name, value):
            return self.declare_parameter(name, value, ParameterDescriptor(read_only=True))

        p("left_port", "/dev/ttyAMA3")
        p("right_port", "/dev/ttyAMA4")
        p("baud", 115200)

        # Геометрия шасси (нужна только для раскладки cmd_vel по бортам)
        p("wheel_separation", 0.48)

        # Калибровка одометрии по оборотам выходного вала
        p("tacho_counts_per_revolution", 2157.0)
        p("distance_per_revolution", 2.011)
        p("odometry_scale", 1.0)
        p("left_odometry_scale", 1.0)
        p("right_odometry_scale", 1.0)

        # Инверсии
        p("invert_left", False)
        p("invert_right", False)
        p("encoder_invert_left", False)
        p("encoder_invert_right", True)
        p("invert_angular", False)

        # Скважность (duty cycle) VESC
        p("duty_min", 0.03)
        p("duty_max", 1.0)
        p("max_linear_velocity", 1.0)
        p("max_angular_velocity", 1.0)
        p("control_rate", 50.0)
        p("telemetry_rate", 20.0)
        p("cmd_timeout", 0.5)
        p("telemetry_stale_timeout", 0.5)

        # Фильтрация скачков тахометра
        p("tacho_jump_margin", 3.0)
        p("min_tacho_jump_threshold", 10.0)
        p("telemetry_pair_max_skew", 0.10)

        # Публикации
        p("publish_odom", True)
        p("odom_topic", "odom/vesc")
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("publish_joint_states", True)
        p("left_wheel_joint", "left_track_joint")
        p("right_wheel_joint", "right_track_joint")
        p("publish_diagnostics", True)

        g = self.get_parameter

        self.left_port = str(g("left_port").value)
        self.right_port = str(g("right_port").value)
        self.baud = int(g("baud").value)

        self.separation = float(g("wheel_separation").value)
        self.tacho_counts_per_revolution = float(g("tacho_counts_per_revolution").value)
        self.distance_per_revolution = float(g("distance_per_revolution").value)
        self.odometry_scale = float(g("odometry_scale").value)
        self.track_scales = {side: float(g(side + "_odometry_scale").value)
                             for side in ("left", "right")}

        self.radius = self.distance_per_revolution / (2.0 * math.pi)

        self.kin_inv_left = -1 if g("invert_left").value else 1
        self.kin_inv_right = -1 if g("invert_right").value else 1
        self.enc_inv_left = -1 if g("encoder_invert_left").value else 1
        self.enc_inv_right = -1 if g("encoder_invert_right").value else 1
        self.invert_angular = bool(g("invert_angular").value)

        self.duty_min = float(g("duty_min").value)
        self.duty_max = float(g("duty_max").value)
        self.max_linear_velocity = float(g("max_linear_velocity").value)
        self.max_angular_velocity = float(g("max_angular_velocity").value)
        self.control_rate = float(g("control_rate").value)
        self.telemetry_rate = float(g("telemetry_rate").value)
        self.cmd_timeout = float(g("cmd_timeout").value)
        self.telemetry_stale_timeout = float(g("telemetry_stale_timeout").value)
        self.tacho_jump_margin = float(g("tacho_jump_margin").value)
        self.min_tacho_jump_threshold = float(g("min_tacho_jump_threshold").value)
        self.telemetry_pair_max_skew = float(g("telemetry_pair_max_skew").value)

        self.pub_odom = bool(g("publish_odom").value)
        self.odom_topic = str(g("odom_topic").value)
        self.odom_frame = str(g("odom_frame").value)
        self.base_frame = str(g("base_frame").value)
        self.pub_js = bool(g("publish_joint_states").value)
        self.left_joint = str(g("left_wheel_joint").value)
        self.right_joint = str(g("right_wheel_joint").value)
        self.pub_diag = bool(g("publish_diagnostics").value)

        self._validate_params()

        self.distance_per_tacho_count = (
            self.distance_per_revolution / self.tacho_counts_per_revolution
        )
        self.rad_per_tacho_count = (2.0 * math.pi) / self.tacho_counts_per_revolution

        self.get_logger().info("=" * 60)
        self.get_logger().info("КОНФИГУРАЦИЯ ПРИВОДА (DUTY CYCLE)")
        self.get_logger().info(f"  Радиус: {self.radius:.4f} м | База: {self.separation:.4f} м")
        self.get_logger().info(f"  Инверсия L/R: {self.kin_inv_left}/{self.kin_inv_right} | "
                               f"invert_angular: {self.invert_angular}")
        self.get_logger().info(f"  Макс. линейная: {self.max_linear_velocity:.2f} м/с")
        self.get_logger().info(f"  Скважность: min={self.duty_min:.3f} max={self.duty_max:.3f}")
        self.get_logger().info(
            f"  Одометрия: {self.tacho_counts_per_revolution:.2f} тиков/оборот, "
            f"{self.distance_per_revolution:.4f} м/оборот "
            f"({self.distance_per_tacho_count * 1000.0:.5f} мм/тик)"
        )
        self.get_logger().info(
            "  Курс (yaw) по гусеницам НЕ считается — его даёт IMU (robot_localization)."
        )
        self.get_logger().info("=" * 60)

        # ----------------------------------------------------------- драйверы
        self.left = VescDriver("left", self.left_port, self.baud, self.get_logger())
        self.right = VescDriver("right", self.right_port, self.baud, self.get_logger())
        self.left.start()
        self.right.start()

        # ----------------------------------------------------------- состояние
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.wheels = {
            "left": self._make_wheel_state(),
            "right": self._make_wheel_state(),
        }

        self.trackers = {
            side: Tachometer(
                self.distance_per_tacho_count * self.odometry_scale * self.track_scales[side],
                self.rad_per_tacho_count, sign=sign,
                max_speed=self.max_linear_velocity + self.max_angular_velocity * self.separation / 2,
                jump_margin=self.tacho_jump_margin,
                min_jump_counts=self.min_tacho_jump_threshold,
                stale_timeout=self.telemetry_stale_timeout)
            for side, sign in (("left", self.enc_inv_left), ("right", self.enc_inv_right))
        }
        self.odometry_session = str(uuid.uuid4())
        self.track_pubs = {
            side: self.create_publisher(TrackTicks, "kolesa/track_" + side, 50)
            for side in ("left", "right")
        }
        self.last_track_stamps = {"left": None, "right": None}
        self.last_published_pair = (None, None)
        self.distance_pubs = {
            name: self.create_publisher(Float64, "kolesa/distance_" + name, 10)
            for name in ("left", "right", "center")
        }

        # ----------------------------------------------------------- топики
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        if self.pub_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        if self.pub_js:
            self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        if self.pub_diag:
            self.diag_pub = self.create_publisher(
                DiagnosticArray, "kolesa/diagnostics", qos_profile_sensor_data,
            )

        # ----------------------------------------------------------- таймеры
        self.create_timer(1.0 / self.control_rate, self._control_tick)
        self.create_timer(1.0 / self.telemetry_rate, self._telemetry_tick)

        self.get_logger().info("kolesa_control: signed tachometer + hardware travel counter")

    # ------------------------------------------------------------- параметры
    def _validate_params(self):
        positive = (
            self.separation, self.tacho_counts_per_revolution, self.distance_per_revolution,
            self.odometry_scale, *self.track_scales.values(), self.max_linear_velocity, self.max_angular_velocity,
            self.control_rate, self.telemetry_rate, self.cmd_timeout,
            self.telemetry_stale_timeout, self.tacho_jump_margin, self.telemetry_pair_max_skew,
        )
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("Geometry, scales, rates and timeouts must be finite and positive")
        if (not math.isfinite(self.min_tacho_jump_threshold)
                or self.min_tacho_jump_threshold < 0):
            raise ValueError("min_tacho_jump_threshold must be finite and nonnegative")
        if not (math.isfinite(self.duty_min) and math.isfinite(self.duty_max)
                and 0 <= self.duty_min < self.duty_max <= 1):
            raise ValueError("Duty limits must satisfy 0 <= duty_min < duty_max <= 1")
        if self.telemetry_pair_max_skew > self.telemetry_stale_timeout:
            raise ValueError("Pair skew must not exceed telemetry_stale_timeout")

    def _make_wheel_state(self):
        return {
            "pos": 0.0, "distance": 0.0, "travel": 0.0, "speed": 0.0, "omega": 0.0,
            "measurement_valid": False, "counter_valid": False, "counter_error": "waiting for tachometers",
            "signed_counts": 0,
            "raw_tacho": None, "raw_tacho_abs": None,
            "prev_tacho": None,
            "initial_tacho": None,
            "total_abs_counts": 0,
            "last_delta_counts": 0, "erpm": 0.0,
            "duty_measured": 0.0, "duty_target": 0.0,
            "voltage": 0.0, "current_motor": 0.0, "temp_fet": 0.0,
            "fault": 0,
            "last_rx_time": None, "last_tacho_time": None,
            "telemetry_age": float("inf"), "stale": True,
        }

    # ------------------------------------------------------------- callbacks
    def _on_cmd_vel(self, msg: Twist):
        finite = math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)
        self.cmd_v = float(msg.linear.x) if finite else 0.0
        self.cmd_w = float(msg.angular.z) if finite else 0.0
        self.last_cmd_time = self.get_clock().now()

    def _control_tick(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

        if not 0 <= dt <= self.cmd_timeout or not self._both_tracks_valid():
            v = 0.0
            w = 0.0
        else:
            v = self.cmd_v
            w = self.cmd_w

        v = max(-self.max_linear_velocity, min(self.max_linear_velocity, v))
        w = max(-self.max_angular_velocity, min(self.max_angular_velocity, w))

        if self.invert_angular:
            w = -w

        half_sep = self.separation / 2.0

        v_left_target = v - (w * half_sep)
        v_right_target = v + (w * half_sep)

        v_left_final = v_left_target * self.kin_inv_left
        v_right_final = v_right_target * self.kin_inv_right

        duty_left = velocity_to_duty_cycle(
            v_left_final, self.max_linear_velocity, self.duty_min, self.duty_max,
        )
        duty_right = velocity_to_duty_cycle(
            v_right_final, self.max_linear_velocity, self.duty_min, self.duty_max,
        )

        self.wheels["left"]["duty_target"] = duty_left
        self.wheels["right"]["duty_target"] = duty_right

        self.left.set_duty(duty_left)
        self.right.set_duty(duty_right)

    def _telemetry_tick(self):
        self.left.request_telemetry()
        self.right.request_telemetry()

        tl = self.left.get_telemetry()
        tr = self.right.get_telemetry()

        self._update_wheel_from_tacho("left", tl)
        self._update_wheel_from_tacho("right", tr)
        self._update_stale_state("left")
        self._update_stale_state("right")

        self._publish_track_samples()
        pair = tuple(self.wheels[side]["last_rx_time"] for side in ("left", "right"))
        valid = self._both_tracks_valid()
        new_pair = valid and all(t != p for t, p in zip(pair, self.last_published_pair))
        if self.pub_odom and (new_pair or not valid):
            self._publish_odom()
        if new_pair:
            self.last_published_pair = pair
            for side in ("left", "right"):
                self.distance_pubs[side].publish(Float64(data=self.wheels[side]["distance"]))
            distance = (self.wheels["left"]["distance"] + self.wheels["right"]["distance"]) / 2
            self.distance_pubs["center"].publish(Float64(data=distance))
            if self.pub_js:
                self._publish_joint_states()
        if self.pub_diag:
            self._publish_diagnostics()

    def _update_wheel_from_tacho(self, side, telemetry):
        st = self.wheels[side]
        if telemetry is None:
            return
        rx_time = telemetry.get("_rx_time")
        # Never manufacture a new receipt time for a cached telemetry dict.
        if (not isinstance(rx_time, (int, float)) or not math.isfinite(rx_time)
                or rx_time > time.monotonic()
                or (st["last_rx_time"] is not None and rx_time <= st["last_rx_time"])):
            return
        tracker = self.trackers[side]
        st["last_rx_time"] = rx_time
        try:
            raw_tacho = telemetry.get("tachometer")
            raw_abs = telemetry.get("tachometer_abs")
            for key, source in (("erpm", "rpm"), ("duty_measured", "duty"),
                                ("voltage", "v_in"), ("current_motor", "current_motor"),
                                ("temp_fet", "temp_fet")):
                value = telemetry.get(source)
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("Incomplete/nonfinite VESC telemetry: " + source)
                st[key] = float(value)
            fault = telemetry.get("fault_code")
            if type(fault) is not int:
                raise ValueError("Missing VESC fault code")
            st["fault"] = fault
            accepted = tracker.update(raw_tacho, raw_abs, rx_time)
            st["raw_tacho"], st["raw_tacho_abs"] = raw_tacho, raw_abs
        except (TypeError, ValueError) as error:
            st["measurement_valid"] = st["counter_valid"] = False
            st["counter_error"] = str(error)
            st["speed"] = st["omega"] = 0.0
            return
        st.update(
            initial_tacho=tracker.initial, signed_counts=tracker.counts,
            total_abs_counts=tracker.travel_counts, last_delta_counts=tracker.delta_counts,
            pos=tracker.angle, distance=tracker.distance, travel=tracker.travel,
            speed=tracker.speed, omega=tracker.omega,
            measurement_valid=accepted and not tracker.fault and not fault,
            counter_valid=accepted and tracker.velocity_valid and not fault,
            counter_error=tracker.fault or ("VESC fault" if fault else ""),
        )
        if tracker.fault:
            st["speed"] = st["omega"] = 0.0

    def _update_stale_state(self, side):
        st = self.wheels[side]
        if st["last_rx_time"] is None:
            st["telemetry_age"] = float("inf")
            st["stale"] = True
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["erpm"] = 0.0
            st["duty_measured"] = 0.0
            return

        age = time.monotonic() - st["last_rx_time"]
        st["telemetry_age"] = age
        if not 0 <= age <= self.telemetry_stale_timeout:
            st["stale"] = True
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["erpm"] = 0.0
            st["duty_measured"] = 0.0
        else:
            st["stale"] = False

    # ------------------------------------------------------------- публикации
    def _both_tracks_valid(self):
        left = self.wheels["left"]
        right = self.wheels["right"]
        now = time.monotonic()
        return (
            all(st["counter_valid"] and st["last_rx_time"] is not None
                and 0 <= now - st["last_rx_time"] <= self.telemetry_stale_timeout
                for st in (left, right))
            and abs(left["last_rx_time"] - right["last_rx_time"]) <= self.telemetry_pair_max_skew
        )

    def _publish_track_samples(self):
        # Publish independent timestamps: averaging/asynchronous pairing belongs
        # in counter_odometry, not in the hardware node.
        for side in ("left", "right"):
            state = self.wheels[side]
            stamp = state["last_rx_time"]
            valid = bool(state["measurement_valid"] and not state["stale"])
            if valid and stamp == self.last_track_stamps[side]:
                continue
            msg = TrackTicks()
            ros_ns = self.get_clock().now().nanoseconds
            age = max(0.0, time.monotonic() - stamp) if stamp is not None else 0.0
            msg.header.stamp = Time(nanoseconds=max(0, ros_ns - int(age * 1e9))).to_msg()
            msg.header.frame_id = self.base_frame
            msg.session_id = self.odometry_session
            msg.position_ticks = self.trackers[side].counts
            msg.travel_ticks = self.trackers[side].travel_counts
            msg.meters_per_tick = self.trackers[side].meters_per_count
            msg.valid = valid
            self.track_pubs[side].publish(msg)
            self.last_track_stamps[side] = stamp

    def _measurement_stamp(self):
        now = time.monotonic()
        received = min(self.wheels[side]["last_rx_time"] for side in ("left", "right"))
        stamp = self.get_clock().now().nanoseconds - int(max(0.0, now - received) * 1e9)
        return Time(nanoseconds=max(0, stamp)).to_msg()

    def _publish_odom(self):
        """
        /odom/vesc: ТОЛЬКО линейная скорость центра робота.

        Поза и угловая скорость не измеряются (ковариация 1e6). Курс даёт
        BNO086; локальные X/Y считает counter_odometry из /kolesa/track_* + IMU.
        """
        left = self.wheels["left"]
        right = self.wheels["right"]

        msg = Odometry()
        valid = self._both_tracks_valid()
        msg.header.stamp = self._measurement_stamp() if valid else self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        v_center = 0.5 * (left["speed"] + right["speed"]) if valid else 0.0

        msg.twist.twist.linear.x = v_center

        # Все компоненты — «не измерено», кроме vx.
        twist_cov = [0.0] * 36
        for i in range(6):
            twist_cov[i * 6 + i] = UNMEASURED_COV
        twist_cov[0] = 0.01 if valid else UNMEASURED_COV  # vx: ±0.1 м/с
        msg.twist.covariance = twist_cov

        pose_cov = [0.0] * 36
        for i in range(6):
            pose_cov[i * 6 + i] = UNMEASURED_COV
        msg.pose.covariance = pose_cov
        msg.pose.pose.orientation.w = 1.0

        self.odom_pub.publish(msg)

    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = self._measurement_stamp()
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.wheels["left"]["pos"], self.wheels["right"]["pos"]]
        js.velocity = [self.wheels["left"]["omega"], self.wheels["right"]["omega"]]
        self.js_pub.publish(js)

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        self._append_wheel_diag(arr, "left", "Левая гусеница", self.left)
        self._append_wheel_diag(arr, "right", "Правая гусеница", self.right)
        self._append_odom_diag(arr)
        self.diag_pub.publish(arr)

    def _append_wheel_diag(self, arr, side, display_name, driver):
        st = self.wheels[side]
        status = DiagnosticStatus()
        status.name = display_name
        status.hardware_id = driver.port

        if not driver.connected:
            status.level = DiagnosticStatus.WARN
            status.message = "Нет соединения с VESC"
        elif st["stale"]:
            status.level = DiagnosticStatus.WARN
            status.message = "Телеметрия устарела"
        elif st["counter_error"]:
            status.level = DiagnosticStatus.ERROR
            status.message = st["counter_error"]
        elif st["fault"]:
            status.level = DiagnosticStatus.ERROR
            status.message = f"VESC fault code {st['fault']}"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Норма"

        total_abs_revolutions = (
            st["total_abs_counts"] / self.tacho_counts_per_revolution
        )

        kv = status.values.append
        kv(KeyValue(key="connected", value=str(driver.connected)))
        kv(KeyValue(
            key="telemetry_age_s",
            value="inf" if math.isinf(st["telemetry_age"]) else f"{st['telemetry_age']:.3f}",
        ))
        kv(KeyValue(key="raw_tachometer", value=str(st["raw_tacho"])))
        kv(KeyValue(key="raw_tachometer_abs", value=str(st["raw_tacho_abs"])))
        kv(KeyValue(key="initial_tacho", value=str(st["initial_tacho"])))
        kv(KeyValue(key="counter_valid", value=str(st["counter_valid"])))
        kv(KeyValue(key="signed_counts", value=str(st["signed_counts"])))
        kv(KeyValue(key="travel_m", value=f"{st['travel']:.3f}"))
        kv(KeyValue(key="last_delta_counts", value=str(st["last_delta_counts"])))
        kv(KeyValue(key="total_abs_counts", value=str(st["total_abs_counts"])))
        kv(KeyValue(key="total_abs_revolutions", value=f"{total_abs_revolutions:.2f}"))
        kv(KeyValue(key="distance_m", value=f"{st['distance']:.3f}"))
        kv(KeyValue(key="duty_target", value=f"{st['duty_target']:.3f}"))
        kv(KeyValue(key="duty_measured", value=f"{st['duty_measured']:.3f}"))
        kv(KeyValue(key="erpm", value=f"{st['erpm']:.1f}"))
        kv(KeyValue(key="speed_m_s", value=f"{st['speed']:.3f}"))
        kv(KeyValue(key="omega_rad_s", value=f"{st['omega']:.3f}"))
        kv(KeyValue(key="voltage_v", value=f"{st['voltage']:.2f}"))
        kv(KeyValue(key="current_motor_a", value=f"{st['current_motor']:.2f}"))
        kv(KeyValue(key="temp_fet_c", value=f"{st['temp_fet']:.1f}"))
        kv(KeyValue(key="fault", value=str(st["fault"])))
        arr.status.append(status)

    def _append_odom_diag(self, arr):
        left = self.wheels["left"]
        right = self.wheels["right"]

        status = DiagnosticStatus()
        status.name = "Одометрия VESC (скорость и путь)"
        status.hardware_id = "kolesa_control/odom"

        if not self._both_tracks_valid():
            status.level = DiagnosticStatus.WARN
            status.message = "Нет данных (ожидание телеметрии обоих бортов)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Норма"

        v_center = 0.5 * (left["speed"] + right["speed"])
        d_center = 0.5 * (left["distance"] + right["distance"])
        # Оценка скорости поворота по разности бортов — ТОЛЬКО для контроля
        # пробуксовки (сравнить с гироскопом). В одометрию не идёт.
        w_tracks_est = (right["speed"] - left["speed"]) / self.separation

        status.values.append(KeyValue(key="v_center_m_s", value=f"{v_center:.3f}"))
        status.values.append(KeyValue(key="distance_center_m", value=f"{d_center:.3f}"))
        status.values.append(KeyValue(
            key="track_yaw_rate_est_rad_s_(diag_only)", value=f"{w_tracks_est:.3f}",
        ))
        arr.status.append(status)

    # ------------------------------------------------------------- завершение
    def shutdown(self):
        try:
            self.left.set_duty(0.0)
            self.right.set_duty(0.0)
        except Exception:  # noqa: BLE001
            pass
        for drv in (self.left, self.right):
            try:
                drv.stop()
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = KolesaControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка узла по сигналу пользователя")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
