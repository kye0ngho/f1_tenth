import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_srvs.srv import Trigger


class MapServerNode(Node):
    """
    PGM + YAML 지도 파일을 로드해 /map (OccupancyGrid)으로 퍼블리시.

    particle_filter_node가 /map 토픽을 요구하므로 실차 배포 시 필수.
    ROS2 nav2_map_server 대신 의존성 없이 동작하는 경량 구현.

    YAML 형식:
      image: map.pgm
      resolution: 0.05
      origin: [-10.0, -10.0, 0.0]
      negate: 0
      occupied_thresh: 0.65
      free_thresh: 0.196

    서비스: ~/reload — 지도 파일 재로드
    """

    def __init__(self):
        super().__init__('map_server_node')

        self.declare_parameter('map_yaml', '/sim_ws/src/localization/maps/map.yaml')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)

        self.map_yaml = self.get_parameter('map_yaml').value
        map_topic = self.get_parameter('map_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate').value)

        # Latched QoS (새 구독자에게 즉시 전달)
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_msg = None
        self.pub = self.create_publisher(OccupancyGrid, map_topic, qos)
        self.create_service(Trigger, '~/reload', self._srv_reload)
        self.create_timer(1.0 / rate, self._publish)

        self._load()

        self.get_logger().info('map_server_node started')
        self.get_logger().info(f'  map_yaml : {self.map_yaml}')

    def _load(self):
        if not os.path.exists(self.map_yaml):
            self.get_logger().error(f'지도 YAML 없음: {self.map_yaml}')
            self.get_logger().warn('  → /sim_ws/src/localization/maps/map.yaml 경로 확인')
            return False

        try:
            import yaml
        except ImportError:
            self.get_logger().error('PyYAML 미설치 (pip install pyyaml)')
            return False

        with open(self.map_yaml, 'r') as f:
            meta = yaml.safe_load(f)

        img_path = meta.get('image', '')
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(self.map_yaml), img_path)

        if not os.path.exists(img_path):
            self.get_logger().error(f'지도 이미지 없음: {img_path}')
            return False

        # PIL로 PGM 로드
        try:
            from PIL import Image as PILImage
            img = PILImage.open(img_path).convert('L')
            data = np.array(img, dtype=np.float32)
        except ImportError:
            # Pillow 없으면 numpy raw read 시도
            self.get_logger().warn('Pillow 미설치, numpy raw read 시도')
            data = self._read_pgm_raw(img_path)
            if data is None:
                return False

        resolution = float(meta.get('resolution', 0.05))
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        negate = int(meta.get('negate', 0))
        occ_thresh = float(meta.get('occupied_thresh', 0.65))
        free_thresh = float(meta.get('free_thresh', 0.196))

        if negate:
            data = 255.0 - data

        # 픽셀값 → 점유 확률 → ROS OccupancyGrid (-1=unknown, 0=free, 100=occ)
        occ = np.full(data.shape, -1, dtype=np.int8)
        prob = 1.0 - data / 255.0
        occ[prob < free_thresh] = 0
        occ[prob > occ_thresh] = 100

        # ROS: y축 반전 (PGM은 top-left origin)
        occ = np.flipud(occ)

        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.info = MapMetaData()
        grid.info.resolution = resolution
        grid.info.width = int(data.shape[1])
        grid.info.height = int(data.shape[0])
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.orientation.w = 1.0
        grid.data = occ.flatten().tolist()

        self.map_msg = grid
        self.get_logger().info(
            f'  지도 로드: {grid.info.width}×{grid.info.height} '
            f'res={resolution}m/px'
        )
        return True

    def _read_pgm_raw(self, path):
        try:
            with open(path, 'rb') as f:
                magic = f.readline().strip()
                if magic not in (b'P5', b'P2'):
                    return None
                while True:
                    line = f.readline().strip()
                    if not line.startswith(b'#'):
                        w, h = map(int, line.split())
                        break
                maxval = int(f.readline().strip())
                raw = np.frombuffer(f.read(), dtype=np.uint8)
                return raw.reshape((h, w)).astype(np.float32) / maxval * 255
        except Exception as e:
            self.get_logger().error(f'PGM 읽기 실패: {e}')
            return None

    def _publish(self):
        if self.map_msg is None:
            return
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)

    def _srv_reload(self, _, response):
        ok = self._load()
        response.success = ok
        response.message = '지도 재로드 완료' if ok else '재로드 실패'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
