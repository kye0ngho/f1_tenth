import csv
import math
import os

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger


# ────────────────────────────────────────────────────────────────────── #
# 추출 코어 (ROS 무관 — 단독 테스트 가능)
# ────────────────────────────────────────────────────────────────────── #
def load_occupancy(map_yaml_path):
    """지도 yaml+이미지 → (free_mask[row,col], resolution, origin_xy, height)"""
    import yaml
    from PIL import Image

    with open(map_yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    img_path = meta.get('image', '')
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)

    img = Image.open(img_path).convert('L')
    data = np.array(img, dtype=np.float32)

    if int(meta.get('negate', 0)):
        data = 255.0 - data

    free_thresh = float(meta.get('free_thresh', 0.196))
    prob = 1.0 - data / 255.0
    free = prob < free_thresh

    resolution = float(meta.get('resolution', 0.05))
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    return free, resolution, (float(origin[0]), float(origin[1])), data.shape[0]


def extract_centerline_pixels(free, seed_rc, ridge_eps_px=1.0,
                              min_clearance_px=2.0):
    """
    회랑 중심선 픽셀 추출.

    레이스 루프는 항상 '안쪽 섬'을 감싸고 돈다. 벽 성분들의 내부를 채워
    (fill_holes) 지도 테두리에 닿지 않는 최대 면적 solid = 안쪽 섬으로 잡고,
    '섬까지 거리 == 나머지 벽까지 거리'인 등거리 능선을 중심선으로 취한다.
    트랙 밖 개활지는 등거리 조건을 만족하지 못해 자동 배제된다.
    """
    if not free[seed_rc]:
        raise ValueError('seed가 자유공간 위에 있지 않습니다')

    wall = ~free
    filled = ndimage.binary_fill_holes(wall)
    lbl, n = ndimage.label(filled)
    if n == 0:
        raise ValueError('지도에 벽이 없습니다')

    border_ids = set(lbl[0, :]) | set(lbl[-1, :]) \
        | set(lbl[:, 0]) | set(lbl[:, -1])
    border_ids.discard(0)
    areas = ndimage.sum(np.ones(lbl.shape), lbl, index=range(1, n + 1))
    candidates = [(areas[i - 1], i) for i in range(1, n + 1)
                  if i not in border_ids]
    if not candidates:
        raise ValueError('안쪽 섬 후보가 없습니다 — 루프 트랙이 아닌 지도')
    island = lbl == max(candidates)[1]
    other = wall & ~island

    d_island = ndimage.distance_transform_edt(~island)
    d_other = ndimage.distance_transform_edt(~other)

    # seed에서의 벽 거리로 회랑 반폭을 추정해 원거리 스퓨리어스 능선 제거
    half_w = min(d_island[seed_rc], d_other[seed_rc])
    max_dist = max(half_w * 4.0, min_clearance_px * 4.0)

    ridge = free \
        & (np.abs(d_island - d_other) <= ridge_eps_px) \
        & (np.minimum(d_island, d_other) >= min_clearance_px) \
        & (d_island <= max_dist)

    rows, cols = np.nonzero(ridge)
    return np.stack([rows, cols], axis=1)


def order_loop(points, start_rc=None):
    """능선 픽셀 점군을 최근접 이웃 그리디로 한 바퀴 순서화."""
    pts = points.astype(np.float64)
    n = len(pts)
    visited = np.zeros(n, dtype=bool)
    if start_rc is not None:
        d2s = np.sum((pts - np.asarray(start_rc, dtype=np.float64)) ** 2, axis=1)
        first = int(np.argmin(d2s))
    else:
        first = 0
    order = [first]
    visited[first] = True
    for _ in range(n - 1):
        cur = pts[order[-1]]
        d2 = np.sum((pts - cur) ** 2, axis=1)
        d2[visited] = np.inf
        nxt = int(np.argmin(d2))
        # 능선이 갈라져 멀리 점프하면 루프가 끝난 것 — 잔가지 무시
        if d2[nxt] > 20.0 ** 2:
            break
        order.append(nxt)
        visited[nxt] = True
    return pts[order]


def pixels_to_world(ordered_rc, resolution, origin_xy, img_height):
    """픽셀 (row,col) → 월드 (x,y). PGM은 top-left origin이라 y축 반전."""
    rows = ordered_rc[:, 0]
    cols = ordered_rc[:, 1]
    x = origin_xy[0] + (cols + 0.5) * resolution
    y = origin_xy[1] + (img_height - rows - 0.5) * resolution
    return np.stack([x, y], axis=1)


def resample(points_xy, spacing):
    """호 길이 기준 균일 간격 재샘플링 (루프 닫힘 가정)."""
    closed = np.vstack([points_xy, points_xy[:1]])
    seg = np.hypot(np.diff(closed[:, 0]), np.diff(closed[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n_out = max(int(total / spacing), 8)
    s_new = np.linspace(0.0, total, n_out, endpoint=False)
    x = np.interp(s_new, s, closed[:, 0])
    y = np.interp(s_new, s, closed[:, 1])
    return np.stack([x, y], axis=1)


def extract_centerline(map_yaml_path, seed_xy=(0.0, 0.0), spacing=0.25,
                       ridge_eps_px=1.0, min_clearance_px=2.0,
                       seed_yaw=0.0):
    """지도 → 중심선 waypoint (x, y, yaw) 배열. 최상위 진입점."""
    free, res, origin, h = load_occupancy(map_yaml_path)
    seed_col = int((seed_xy[0] - origin[0]) / res)
    seed_row = int(h - 1 - (seed_xy[1] - origin[1]) / res)
    ridge_px = extract_centerline_pixels(
        free, (seed_row, seed_col), ridge_eps_px, min_clearance_px)
    ordered = order_loop(ridge_px, start_rc=(seed_row, seed_col))
    world = pixels_to_world(ordered, res, origin, h)
    wps = resample(world, spacing)

    # 루프 진행 방향을 차량 스폰 heading과 일치시킴 —
    # 반대면 컨트롤러가 '전방' 포인트를 반대편 회랑에서 찾는다
    first_seg = wps[1] - wps[0]
    heading = np.array([math.cos(seed_yaw), math.sin(seed_yaw)])
    if float(np.dot(first_seg, heading)) < 0.0:
        wps = np.flipud(wps)

    yaws = np.zeros(len(wps))
    for i in range(len(wps)):
        nxt = wps[(i + 1) % len(wps)]
        yaws[i] = math.atan2(nxt[1] - wps[i][1], nxt[0] - wps[i][0])
    return np.column_stack([wps, yaws])


# ────────────────────────────────────────────────────────────────────── #
# ROS 노드
# ────────────────────────────────────────────────────────────────────── #
class CenterlineExtractorNode(Node):
    """
    점유격자 지도에서 트랙 중심선을 추출해 waypoints CSV로 저장.

    지도만 있으면 주행 없이 waypoint를 자동 생성한다 (시뮬·실차 공용).
    이후 race_line_optimizer / velocity_profile 체인에 그대로 연결된다.

    서비스: ~/extract — 추출 실행 후 CSV 저장
    출력:   /planning/centerline (Path — RViz 확인용)
    """

    def __init__(self):
        super().__init__('centerline_extractor_node')

        self.declare_parameter('map_yaml', '/sim_ws/src/localization/maps/map.yaml')
        self.declare_parameter('output_csv', '/sim_ws/src/planning/waypoints/waypoints.csv')
        self.declare_parameter('path_topic', '/planning/centerline')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoint_spacing', 0.25)
        self.declare_parameter('default_speed', 1.0)
        self.declare_parameter('seed_x', 0.0)   # 트랙 회랑 위의 한 점 (차량 스폰)
        self.declare_parameter('seed_y', 0.0)
        self.declare_parameter('seed_yaw', 0.0)  # 스폰 heading — 루프 진행 방향 결정
        self.declare_parameter('ridge_eps_px', 1.0)
        self.declare_parameter('min_clearance_px', 2.0)
        self.declare_parameter('auto_extract_on_start', False)

        self.map_yaml = self.get_parameter('map_yaml').value
        self.output_csv = self.get_parameter('output_csv').value
        path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.spacing = float(self.get_parameter('waypoint_spacing').value)
        self.default_speed = float(self.get_parameter('default_speed').value)
        self.seed = (float(self.get_parameter('seed_x').value),
                     float(self.get_parameter('seed_y').value))
        self.seed_yaw = float(self.get_parameter('seed_yaw').value)
        self.ridge_eps = float(self.get_parameter('ridge_eps_px').value)
        self.min_clear = float(self.get_parameter('min_clearance_px').value)
        auto = bool(self.get_parameter('auto_extract_on_start').value)

        self.waypoints = None
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.create_service(Trigger, '~/extract', self._srv_extract)
        self.create_timer(2.0, self._publish_path)

        self.get_logger().info('centerline_extractor_node started')
        self.get_logger().info(f'  map_yaml         : {self.map_yaml}')
        self.get_logger().info(f'  output_csv       : {self.output_csv}')
        self.get_logger().info(f'  waypoint_spacing : {self.spacing} m')
        self.get_logger().info(f'  seed             : {self.seed}')
        self.get_logger().info(f'  auto_extract     : {auto}')

        if auto:
            self._extract()

    def _extract(self):
        wps = extract_centerline(
            self.map_yaml, self.seed, self.spacing,
            self.ridge_eps, self.min_clear, self.seed_yaw)
        self.waypoints = wps
        self._save_csv(wps)
        self.get_logger().info(
            f'중심선 추출 완료: {len(wps)}개 waypoint → {self.output_csv}')
        return len(wps)

    def _save_csv(self, wps):
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        with open(self.output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw', 'speed'])
            for x, y, yaw in wps:
                writer.writerow([round(x, 4), round(y, 4),
                                 round(yaw, 6), self.default_speed])

    def _publish_path(self):
        if self.waypoints is None:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y, yaw in self.waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = self.default_speed
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _srv_extract(self, _, response):
        try:
            n = self._extract()
            response.success = True
            response.message = f'{n}개 waypoint 추출 → {self.output_csv}'
        except Exception as e:
            response.success = False
            response.message = f'추출 실패: {e}'
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CenterlineExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
