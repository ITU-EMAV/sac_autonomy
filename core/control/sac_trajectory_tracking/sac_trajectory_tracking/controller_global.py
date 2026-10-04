import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

import numpy as np


class Controller(Node):
    """Pure pursuit tabanli yol takip kontrolcusu.

    Duzeltmeler:
      * find_lookahead_point() artik gercekten kullaniliyor (sabit indeks yerine).
      * En yakin NODE degil, en yakin SEGMENT uzerindeki izdusum baz aliniyor.
      * Lookahead noktasi son segment uzerinde interpolasyonla bulunuyor -> Ld ayrik degil.
      * alpha [-pi, pi] araligina sariliyor.
      * delta = atan(L * kappa) → cmd_vel.angular.z direksiyon acisi [rad].
      * Ackermann direksiyon limiti ile kappa doyuruluyor.
      * Goal kontrolu indeks yerine kalan yol uzunluguna gore yapiliyor.
      * Lookahead noktasi RViz'de MarkerArray olarak yayinlaniyor.
      * Path egrileğine gore hiz: v <= sqrt(a_lat_max / |kappa|) (a_lat_max=0.5).
      * Curvature lookahead gain varsayilan 0 (Ld egrilekten bagimsiz).
    """

    def __init__(self):
        super().__init__("controller")
        callback_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------ #
        # Parametreler
        # ------------------------------------------------------------------ #
        self.declare_parameter("linear_velocity", 1.0)      # m/s
        self.declare_parameter("wheelbase", 1.7)            # m
        self.declare_parameter("k_ld", 1.2)                 # s  Ld = k_ld * v
        # min dusuk tutulmali; yoksa dusuk hizda Ld hep min'e clip olur
        self.declare_parameter("min_lookahead", 1.5)        # m
        self.declare_parameter("max_lookahead", 6.0)        # m
        # Curvature Ld kazanci kapali (0); egrilek sadece hiz limiti icin kullanilir.
        self.declare_parameter("curvature_gain", 0.0)       # Ld /= (1+g*|kappa|) — su an 0
        self.declare_parameter("curvature_window", 3.0)     # m  lokal egri ornekleme
        self.declare_parameter("a_lat_max", 0.5)            # m/s^2  max yanal ivme
        self.declare_parameter("goal_tolerance", 0.5)       # m
        self.declare_parameter("slow_down_distance", 3.0)   # m
        self.declare_parameter("max_steer_deg", 23.0)       # deg (plant right limit)
        self.declare_parameter("path_timeout", 50.0)         # s
        self.declare_parameter("control_period", 0.05)      # s
        self.declare_parameter("global_frame", "map")

        gp = self.get_parameter
        self.linear_velocity = gp("linear_velocity").value
        self.wheelbase = gp("wheelbase").value
        self.k_ld = gp("k_ld").value
        self.min_ld = gp("min_lookahead").value
        self.max_ld = gp("max_lookahead").value
        self.curvature_gain = gp("curvature_gain").value
        self.curvature_window = gp("curvature_window").value
        self.a_lat_max = gp("a_lat_max").value
        self.goal_tol = gp("goal_tolerance").value
        self.slow_dist = gp("slow_down_distance").value
        self.max_steer = np.deg2rad(gp("max_steer_deg").value)
        self.path_timeout = gp("path_timeout").value
        self.control_period = gp("control_period").value
        self.global_frame = gp("global_frame").value

        # ------------------------------------------------------------------ #
        # Durum
        # ------------------------------------------------------------------ #
        self.path = []                  # list[PoseStamped]
        self.path_xy = None             # (N,2) ndarray, cache
        self.pose: Pose = None
        self.last_path_time = None      # rclpy.time.Time
        self.speed = 0.0                # odometri hizi [m/s], Ld hesabi icin

        # ------------------------------------------------------------------ #
        # Abonelikler / yayinlar
        # ------------------------------------------------------------------ #
        self.create_subscription(
            Path,
            "/trajectory_planner/trajectory",
            self.path_callback,
            10,
            callback_group=callback_group,
        )

        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odometry_callback,
            10,
            callback_group=callback_group,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/pcl_pose",
            self.loc_cb,
            10,
            callback_group=callback_group,
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/controller/lookahead", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(
            self.control_period, self.controller_callback, callback_group=callback_group
        )

        self.get_logger().info("Pure pursuit controller basladi.")

    # ---------------------------------------------------------------------- #
    # Callback'ler
    # ---------------------------------------------------------------------- #
    def path_callback(self, msg: Path):
        if msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Path '{msg.header.frame_id}' frame'inden geldi, "
                f"'{self.global_frame}' bekleniyordu. Yok sayiliyor."
            )
            return

        if len(msg.poses) < 2:
            self.get_logger().warn("Path 2'den az noktaya sahip, yok sayiliyor.")
            return

        self.path = msg.poses
        self.path_xy = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float
        )
        self.last_path_time = self.get_clock().now()

    def loc_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose.pose

    def odometry_callback(self, msg: Odometry):
        # Gercek hizi sakla; adaptif lookahead bunu kullanir.
        # Lokalizasyon /pcl_pose'dan geliyor; odom sadece hiz icin.
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = float(np.hypot(vx, vy))

    # ---------------------------------------------------------------------- #
    # Path egrileği / projeksiyon
    # ---------------------------------------------------------------------- #
    def project_on_path(self, curr):
        """En yakin segment izdusumu, segment indeksi ve kalan yay uzunlugu."""
        pts = self.path_xy
        a = pts[:-1]
        b = pts[1:]
        ab = b - a
        seg_len2 = np.maximum(np.sum(ab * ab, axis=1), 1e-9)
        t = np.clip(np.sum((curr - a) * ab, axis=1) / seg_len2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        i0 = int(np.argmin(np.linalg.norm(proj - curr, axis=1)))
        start = proj[i0]

        seg_len = np.linalg.norm(ab, axis=1)
        s_remain = float(
            np.linalg.norm(pts[i0 + 1] - start) + seg_len[i0 + 1:].sum()
        )
        return start, i0, s_remain

    def estimate_path_curvature(self, i0, start, window_m=None):
        """Izdusumden window_m boyunca path egrileğini (1/m) tahmin eder."""
        if self.path_xy is None or len(self.path_xy) < 3:
            return 0.0

        if window_m is None:
            window_m = self.curvature_window

        pts = self.path_xy
        samples = [start]
        remaining = float(window_m)
        p_prev = start
        for i in range(i0, len(pts) - 1):
            p_next = pts[i + 1]
            seg = float(np.linalg.norm(p_next - p_prev))
            if seg < 1e-9:
                continue
            if seg >= remaining:
                ratio = remaining / seg
                samples.append(p_prev + ratio * (p_next - p_prev))
                break
            samples.append(p_next)
            remaining -= seg
            p_prev = p_next

        if len(samples) < 3:
            return 0.0

        samples = np.asarray(samples)
        kappas = []
        for j in range(1, len(samples) - 1):
            a, b, c = samples[j - 1], samples[j], samples[j + 1]
            ab = np.linalg.norm(b - a)
            bc = np.linalg.norm(c - b)
            ac = np.linalg.norm(c - a)
            area2 = abs(
                (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            )
            denom = ab * bc * ac
            if denom > 1e-9:
                kappas.append(2.0 * area2 / denom)

        return float(np.mean(kappas)) if kappas else 0.0

    # ---------------------------------------------------------------------- #
    # Lookahead hesabi
    # ---------------------------------------------------------------------- #
    def find_lookahead_point(self, Ld, start=None, i0=None, s_remain=None):
        """Robotun path'e izdusumunden Ld kadar ilerideki noktayi bulur.

        Returns:
            (target_xy (2,), idx, s_remain) veya None
            target_xy : lookahead noktasi (interpolasyonlu)
            idx       : lookahead noktasinin hemen ilerisindeki path indeksi
            s_remain  : izdusumden path sonuna kalan yay uzunlugu [m]
        """
        if self.path_xy is None or self.pose is None:
            return None

        pts = self.path_xy
        if len(pts) < 2:
            return pts[-1], 0, 0.0

        if start is None or i0 is None or s_remain is None:
            curr = np.array([self.pose.position.x, self.pose.position.y])
            start, i0, s_remain = self.project_on_path(curr)

        # Izdusumden itibaren Ld kadar ilerle, son segmentte interpole et
        remaining = float(Ld)
        p_prev = start
        for i in range(i0, len(pts) - 1):
            p_next = pts[i + 1]
            seg = float(np.linalg.norm(p_next - p_prev))
            if seg >= remaining:
                ratio = remaining / seg if seg > 1e-9 else 0.0
                target = p_prev + ratio * (p_next - p_prev)
                return target, i + 1, s_remain
            remaining -= seg
            p_prev = p_next

        # Path Ld'den kisa kaldiysa son noktayi hedefle
        return pts[-1], len(pts) - 1, s_remain

    # ---------------------------------------------------------------------- #
    # Kontrol dongusu
    # ---------------------------------------------------------------------- #
    def controller_callback(self):
        # --- Girdi kontrolleri ---
        if self.path_xy is None or len(self.path) < 2:
            self.get_logger().info("Path yok.", throttle_duration_sec=2.0)
            self.stop()
            return

        if self.pose is None:
            self.get_logger().info("Lokalizasyon yok.", throttle_duration_sec=2.0)
            self.stop()
            return

        if self.last_path_time is not None:
            age = (self.get_clock().now() - self.last_path_time).nanoseconds * 1e-9
            if age > self.path_timeout:
                self.get_logger().warn(
                    f"Path bayat ({age:.2f}s), duruluyor.", throttle_duration_sec=2.0
                )
                self.stop()
                return

        # --- Poz ---
        p = self.pose.position
        o = self.pose.orientation
        yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        curr = np.array([p.x, p.y])

        start, i0, s_remain = self.project_on_path(curr)

        # --- Hedefe varis ---
        if s_remain < self.goal_tol:
            self.get_logger().warn("GOAL REACHED!", throttle_duration_sec=2.0)
            self.stop()
            self.publish_lookahead_marker(self.path_xy[-1], curr, 0.0, reached=True)
            return

        # Lokal path egrileği (1/m) — hiz limiti icin; Ld'ye uygulanmaz (gain=0).
        path_kappa = self.estimate_path_curvature(i0, start)

        # --- Hiza bagli adaptif lookahead (curvature_gain=0 → Ld egrilekten bagimsiz) ---
        # Ld = clip(k_ld * v, min_ld, max_ld)
        # Dusuk hizda (<0.15) odom gurultusu / durma icin komut hizina dus.
        v_for_ld = self.speed if self.speed > 0.15 else self.linear_velocity
        Ld_cmd = self.k_ld * v_for_ld
        if self.curvature_gain > 0.0 and abs(path_kappa) > 1e-6:
            Ld_cmd /= 1.0 + self.curvature_gain * abs(path_kappa)
        Ld_cmd = float(np.clip(Ld_cmd, self.min_ld, self.max_ld))

        res = self.find_lookahead_point(Ld_cmd, start=start, i0=i0, s_remain=s_remain)
        if res is None:
            self.stop()
            return
        target, idx, s_remain = res

        # --- Pure pursuit ---
        d = target - curr
        Ld = float(np.hypot(d[0], d[1]))
        if Ld < 1e-3:
            self.stop()
            return

        alpha = np.arctan2(d[1], d[0]) - yaw
        alpha = float(np.arctan2(np.sin(alpha), np.cos(alpha)))  # [-pi, pi]

        kappa = 2.0 * np.sin(alpha) / Ld

        # Ackermann direksiyon limiti: kappa_max = tan(delta_max) / L
        kappa_max = np.tan(self.max_steer) / self.wheelbase
        kappa = float(np.clip(kappa, -kappa_max, kappa_max))

        # --- Hiz profili ---
        # 1) Hedefe yaklasinca yavasla
        v = self.linear_velocity * min(1.0, s_remain / max(self.slow_dist, 1e-3))
        # 2) Yanal ivme limiti: a_lat = v^2 * |kappa| <= a_lat_max
        #    => v_max = sqrt(a_lat_max / |kappa|)
        if abs(path_kappa) > 1e-6:
            v_curve = float(np.sqrt(self.a_lat_max / abs(path_kappa)))
            v = min(v, v_curve)
        v = float(np.clip(v, 0.0, self.linear_velocity))

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = float(np.arctan(self.wheelbase * kappa))  # delta [rad]
        self.cmd_vel_publisher.publish(twist)

        self.publish_lookahead_marker(target, curr, Ld)

    def stop(self):
        self.cmd_vel_publisher.publish(Twist())

    # ---------------------------------------------------------------------- #
    # Gorsellestirme
    # ---------------------------------------------------------------------- #
    def publish_lookahead_marker(self, target, curr, Ld, reached=False):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns = "lookahead"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.5e9)
            return m

        # 1) Lookahead noktasi (kirmizi kure / hedefe varildiysa yesil)
        sphere = base(0, Marker.SPHERE)
        sphere.pose.position.x = float(target[0])
        sphere.pose.position.y = float(target[1])
        sphere.pose.position.z = 1.5
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5
        if reached:
            sphere.color.r, sphere.color.g, sphere.color.b = 0.1, 1.0, 0.1
        else:
            sphere.color.r, sphere.color.g, sphere.color.b = 1.0, 0.1, 0.1
        sphere.color.a = 1.0
        ma.markers.append(sphere)

        # 2) Robot -> lookahead vektoru (yesil cizgi)
        line = base(1, Marker.LINE_STRIP)
        line.scale.x = 0.08
        line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 1.0, 0.3, 0.9
        for xy in (curr, target):
            pt = Point()
            pt.x = float(xy[0])
            pt.y = float(xy[1])
            pt.z = 0.2
            line.points.append(pt)
        ma.markers.append(line)

        # 3) Lookahead cemberi (mavi, robot merkezli, yaricap = Ld)
        circle = base(2, Marker.LINE_STRIP)
        circle.scale.x = 0.04
        circle.color.r, circle.color.g, circle.color.b, circle.color.a = 0.2, 0.5, 1.0, 0.6
        for th in np.linspace(0.0, 2.0 * np.pi, 48):
            pt = Point()
            pt.x = float(curr[0] + Ld * np.cos(th))
            pt.y = float(curr[1] + Ld * np.sin(th))
            pt.z = 0.05
            circle.points.append(pt)
        ma.markers.append(circle)

        # 4) Ld degeri metni
        text = base(3, Marker.TEXT_VIEW_FACING)
        text.pose.position.x = float(target[0])
        text.pose.position.y = float(target[1])
        text.pose.position.z = 0.9
        text.scale.z = 0.4
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"Ld = {Ld:.2f} m"
        ma.markers.append(text)

        self.marker_publisher.publish(ma)


def main():
    rclpy.init()
    controller = Controller()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
