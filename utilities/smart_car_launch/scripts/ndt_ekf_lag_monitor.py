#!/usr/bin/env python3
"""NDT ile EKF arasindaki farki ve encoder hiz olcegini canli olcer.

Her NDT pozu, EKF pozunun AYNI zaman damgasina interpole edilmis haliyle
karsilastirilir (NDT gecikmesi farka karismaz). Fark EKF'in yonune gore
ayristirilir:

  ileri  > 0 : EKF, NDT'nin GERISINDE (arac yonunde geride kaliyor)
  yanal  > 0 : EKF, NDT'nin sagында degil SOLUNDA ... (NDT - EKF, sol +)

Duz segmentlerde NDT'nin kat ettigi mesafe encoder integraline bolunerek
hiz orani hesaplanir; onerilen speed_scale = mevcut_scale * oran.

Kullanim (bag oynatilirken):
  ros2 run smart_car_launch ndt_ekf_lag_monitor.py --ros-args -p use_sim_time:=true
  ros2 run smart_car_launch ndt_ekf_lag_monitor.py --current-scale 1.0 --csv /tmp/lag.csv
  # EKF pose topic'i yerine map->base_link TF'i ile karsilastir:
  ros2 run smart_car_launch ndt_ekf_lag_monitor.py --tf --ros-args -p use_sim_time:=true

--tf modunda ayrica tf_yas basilir: en son map->base_link TF'inin simdiki
saate gore yasi (EKF 40-50 Hz'de ~0-25 ms olmali).
"""

import argparse
import bisect
import collections
import math
import statistics
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener

HISTORY_SEC = 10.0
# Hiz orani penceresi
RATIO_WINDOW_SEC = 2.0
RATIO_MAX_YAW_CHANGE = math.radians(3.0)
RATIO_MIN_DIST = 2.0
RATIO_MAX_NDT_GAP = 0.5
# Bu sureden uzun NDT boslugu "NDT dustu" sayilir
NDT_GAP_WARN_SEC = 0.3


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LagMonitor(Node):

    def __init__(self, args):
        super().__init__('ndt_ekf_lag_monitor')
        self.current_scale = args.current_scale
        self.use_tf = args.tf
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.tf_misses = 0
        if self.use_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.csv = open(args.csv, 'w', encoding='utf-8') if args.csv else None
        if self.csv:
            self.csv.write('stamp,along_m,lateral_m,yaw_deg,ndt_age_ms,ekf_speed_mps\n')

        # (t, x, y, yaw)
        self.ekf = collections.deque()
        self.ndt = collections.deque()
        # (t, vx)
        self.enc = collections.deque()

        self.window = []          # (along, lateral, yaw, ndt_age) bu periyotta
        self.ratios = collections.deque(maxlen=200)
        self.ndt_gaps = 0
        self.ndt_count = 0
        self.no_ekf_match = 0

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(PoseStamped, args.ekf_topic, self.on_ekf, qos)
        self.create_subscription(PoseWithCovarianceStamped, args.ndt_topic, self.on_ndt, qos)
        self.create_subscription(TwistWithCovarianceStamped, args.twist_topic, self.on_twist, qos)
        self.create_timer(args.period, self.report)

        self.get_logger().info(
            f'NDT={args.ndt_topic} EKF={args.ekf_topic} twist={args.twist_topic} '
            f'mevcut speed_scale={self.current_scale} '
            f'kaynak={"TF " + self.map_frame + "->" + self.base_frame if self.use_tf else "pose topic"}')

    @staticmethod
    def _trim(buf, t_now):
        while buf and buf[0][0] < t_now - HISTORY_SEC:
            buf.popleft()

    def on_ekf(self, msg):
        p = msg.pose
        t = stamp_sec(msg.header.stamp)
        if self.ekf and t <= self.ekf[-1][0]:
            if t < self.ekf[-1][0] - 1.0:   # bag basa sardi
                self.ekf.clear()
            else:
                return
        self.ekf.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        self._trim(self.ekf, t)

    def on_twist(self, msg):
        t = stamp_sec(msg.header.stamp)
        if self.enc and t < self.enc[-1][0]:
            self.enc.clear()
        self.enc.append((t, msg.twist.twist.linear.x))
        self._trim(self.enc, t)

    def ekf_at(self, t):
        """EKF pozunu t anina lineer interpole eder."""
        if len(self.ekf) < 2 or t < self.ekf[0][0] or t > self.ekf[-1][0]:
            return None
        times = [s[0] for s in self.ekf]
        i = bisect.bisect_left(times, t)
        if i == 0:
            return self.ekf[0]
        a, b = self.ekf[i - 1], self.ekf[i]
        if b[0] - a[0] > 0.2:
            return None
        r = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return (t, a[1] + r * (b[1] - a[1]), a[2] + r * (b[2] - a[2]),
                a[3] + r * wrap(b[3] - a[3]))

    def tf_at(self, t):
        """map->base_link TF'ini t aninda sorgular (tf2 interpolasyonu)."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(nanoseconds=int(t * 1e9)))
        except Exception:  # noqa: BLE001  (tf2 istisna tipleri surume gore degisiyor)
            self.tf_misses += 1
            return None
        tl = tr.transform.translation
        return (t, tl.x, tl.y, yaw_of(tr.transform.rotation))

    def tf_age_ms(self, now):
        try:
            tr = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:  # noqa: BLE001
            return float('nan')
        return (now - stamp_sec(tr.header.stamp)) * 1000.0

    def encoder_distance(self, t0, t1):
        """Encoder hizinin [t0, t1] integrali (sifirinci derece tutma)."""
        if not self.enc or self.enc[0][0] > t0 or self.enc[-1][0] < t1 - 0.1:
            return None
        dist = 0.0
        samples = list(self.enc)
        for (ta, va), (tb, _) in zip(samples, samples[1:] + [(t1, 0.0)]):
            lo, hi = max(ta, t0), min(tb, t1)
            if hi > lo:
                dist += va * (hi - lo)
        return dist

    def on_ndt(self, msg):
        p = msg.pose.pose
        t = stamp_sec(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        sample = (t, p.position.x, p.position.y, yaw_of(p.orientation))

        if self.ndt and t < self.ndt[-1][0]:
            self.ndt.clear()
        if self.ndt and t - self.ndt[-1][0] > NDT_GAP_WARN_SEC:
            self.ndt_gaps += 1
        self.ndt.append(sample)
        self._trim(self.ndt, t)
        self.ndt_count += 1

        e = self.tf_at(t) if self.use_tf else self.ekf_at(t)
        if e is None:
            self.no_ekf_match += 1
        else:
            dx, dy = sample[1] - e[1], sample[2] - e[2]
            c, s = math.cos(e[3]), math.sin(e[3])
            along = c * dx + s * dy
            lateral = -s * dx + c * dy
            dyaw = math.degrees(wrap(sample[3] - e[3]))
            age_ms = (now - t) * 1000.0
            self.window.append((along, lateral, dyaw, age_ms))
            if self.csv:
                self.csv.write(f'{t:.3f},{along:.3f},{lateral:.3f},{dyaw:.3f},'
                               f'{age_ms:.1f},{self.ekf_speed():.3f}\n')

        self.update_ratio(sample)

    def update_ratio(self, last):
        # ~RATIO_WINDOW_SEC once gelen NDT pozunu bul
        start = None
        for i in range(len(self.ndt) - 1, 0, -1):
            if self.ndt[i][0] - self.ndt[i - 1][0] > RATIO_MAX_NDT_GAP:
                return
            if last[0] - self.ndt[i - 1][0] >= RATIO_WINDOW_SEC:
                start = self.ndt[i - 1]
                break
        if start is None:
            return
        if abs(wrap(last[3] - start[3])) > RATIO_MAX_YAW_CHANGE:
            return
        enc = self.encoder_distance(start[0], last[0])
        if enc is None or enc < RATIO_MIN_DIST:
            return
        self.ratios.append(math.hypot(last[1] - start[1], last[2] - start[2]) / enc)

    def ekf_speed(self):
        if len(self.ekf) < 10:
            return 0.0
        a, b = self.ekf[-10], self.ekf[-1]
        dt = b[0] - a[0]
        return math.hypot(b[1] - a[1], b[2] - a[2]) / dt if dt > 0 else 0.0

    def report(self):
        w, self.window = self.window, []
        now = self.get_clock().now().nanoseconds * 1e-9
        ekf_age = (now - self.ekf[-1][0]) * 1000.0 if self.ekf else float('nan')
        line = f'v={self.ekf_speed():4.1f}m/s  ekf_yas={ekf_age:5.0f}ms  '
        if self.use_tf:
            line += f'tf_yas={self.tf_age_ms(now):5.0f}ms  '
        if w:
            along = [x[0] for x in w]
            line += (f'ileri={statistics.mean(along):+6.2f}m (max {max(along, key=abs):+6.2f})  '
                     f'yanal={statistics.mean(x[1] for x in w):+5.2f}m  '
                     f'yaw={statistics.mean(x[2] for x in w):+5.2f}deg  '
                     f'ndt_yas={statistics.mean(x[3] for x in w):4.0f}ms  ndt_n={len(w)}')
            if abs(statistics.mean(along)) > 0.5:
                line += '  <-- EKF GERIDE' if statistics.mean(along) > 0 else '  <-- EKF ONDE'
        else:
            line += 'NDT eslesmesi yok'
        line += f'  ndt_bosluk={self.ndt_gaps}'
        if self.ratios:
            ratio = statistics.median(self.ratios)
            line += (f'  hiz_orani={ratio:.3f} (n={len(self.ratios)}) '
                     f'-> speed_scale={self.current_scale * ratio:.3f}')
        print(line, flush=True)

    def destroy_node(self):
        if self.csv:
            self.csv.close()
        if self.ratios:
            ratio = statistics.median(self.ratios)
            print(f'\nSONUC: medyan hiz orani {ratio:.4f} ({len(self.ratios)} pencere), '
                  f'onerilen speed_scale = {self.current_scale * ratio:.4f}')
        print(f'NDT pozu: {self.ndt_count}, {NDT_GAP_WARN_SEC}s ustu bosluk: {self.ndt_gaps}, '
              f'EKF ile eslesmeyen: {self.no_ekf_match}'
              + (f', TF bulunamayan: {self.tf_misses}' if self.use_tf else ''))
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ndt-topic', default='/localization/ndt_localizer/pose_with_cov')
    parser.add_argument('--ekf-topic', default='/localization/ekf_localizer/pose')
    parser.add_argument('--twist-topic', default='/vehicle/twist_with_covariance')
    parser.add_argument('--current-scale', type=float, default=1.0,
                        help='vehicle_twist_converter speed_scale mevcut degeri')
    parser.add_argument('--tf', action='store_true',
                        help='EKF pose topic yerine map->base_link TF ile karsilastir')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--period', type=float, default=1.0, help='rapor periyodu [s]')
    parser.add_argument('--csv', default='', help='her NDT pozu icin satir yazilacak dosya')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = LagMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
