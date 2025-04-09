import rclpy
from rclpy.node import Node
import numpy as np
from shapely.geometry import Point, Polygon, LineString
import visualization_msgs.msg as visualization_msgs
import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import Int32

class FailSafeSim(Node):
    def __init__(self):
        super().__init__('fail_safe_sim')

        # 센서 파라미터 설정 [x, y, theta, range, fov]
        sensor_param = [
            [1, 0, 0, 5, 120],
            [-1, 0, 180, 5, 120],
            [0.5, 0.5, 90, 5, 120],
            [0.5, -0.5, -90, 5, 120],
        ]

        self.sensor_num = len(sensor_param)
        self.sensor_publisher = []
        self.sensor_fail_publisher = []
        self.sensor_range = []
        self.sensor_range_g= []
        for i in range(self.sensor_num):
            self.sensor_publisher.append(
                self.create_publisher(geometry_msgs.PolygonStamped, f'sensor_range_{i}', 10)
            )
            self.sensor_fail_publisher.append(
                self.create_publisher(geometry_msgs.PolygonStamped, f'sensor_fail_{i}', 10)
            )
            x, y, theta, range_, fov = sensor_param[i]
            self.sensor_range.append(
                self.to_poly_msg(self.create_sector(x, y, theta, range_, fov))
            )
            self.sensor_range_g.append(
                self.create_sector(x, y, theta, range_, fov)
            )
        
        # Path 퍼블리셔 추가
        self.path_pub = self.create_publisher(visualization_msgs.MarkerArray, 'paths', 10)
        
        self.subscriber_ = self.create_subscription(
            Int32,
            '/input_command',  # 구독할 토픽명
            self.listener_callback,
            10
        )

        timer_period = 0.1 # 초 단위
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sensor_state = [True] * self.sensor_num

    def listener_callback(self,msgs):
        self.sensor_state[msgs.data] = not self.sensor_state[msgs.data]
    
    def points_in_polygon(self,points, polygon):
        for pt in points:
            point = Point(pt)
            if polygon.contains(point):
                return True
        return False

    def timer_callback(self):
        print(self.sensor_state )
        # 센서 범위 퍼블리시
        for i in range(self.sensor_num):
            if self.sensor_state[i]:
                self.sensor_publisher[i].publish(self.sensor_range[i])
            else:
                self.sensor_fail_publisher[i].publish(self.sensor_range[i])
            

        # Path 퍼블리시
        paths = self.path_sim(v=[1.0, 0.2])
        msgs = visualization_msgs.MarkerArray()

        for i, p in enumerate(paths):
            msgs.markers.append(self.to_path_msg(i, p))

        self.path_pub.publish(msgs)
        check, score = self.cal_score(self.sensor_range ,paths)
        print(check,score)


    def to_path_msg(self, id, line):
        marker = visualization_msgs.Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lines"
        marker.id = id
        marker.type = visualization_msgs.Marker.LINE_STRIP
        marker.action = visualization_msgs.Marker.ADD
        marker.scale.x = 0.01  # 선 두께

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        line = np.array(line.coords)
        for (x, y) in line:
            p = geometry_msgs.Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        return marker

    def to_poly_msg(self, poly):
        polygon_array = np.array(poly.exterior.coords)[:-1]  # 마지막 점 제거 (중복 제거)

        polygon_msg = geometry_msgs.PolygonStamped()
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.header.frame_id = "base_link"

        for point in polygon_array:
            pt = geometry_msgs.Point32()
            pt.x = float(point[0])
            pt.y = float(point[1])
            pt.z = 0.0
            polygon_msg.polygon.points.append(pt)

        return polygon_msg

    def create_sector(self, x, y, theta, radius, angle_deg, resolution=100):
        start_angle = theta - angle_deg / 2
        end_angle = theta + angle_deg / 2

        angles = np.radians(np.linspace(start_angle, end_angle, resolution))
        arc = [(x + radius * np.cos(a), y + radius * np.sin(a)) for a in angles]

        points = [(x, y)] + arc + [(x, y)]
        sector = Polygon(points)

        return sector

    def simulate_path(self, v, w, x0=0, y0=0, theta0=0, dt=0.1, T=10.0):
        """unicycle 모델 기반 path 시뮬레이션"""
        x, y, theta = x0, y0, theta0
        positions = [(x, y)]

        for _ in range(int(T / dt)):
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += w * dt
            positions.append((x, y))

        path = LineString(positions)
        return path

    def path_sim(self, v=[1.0, 0.2], w=[0.1, 0.05], t=5, sampling=100):
        v_samples = np.linspace(v[0]-v[1], v[0]+v[1], sampling)
        w_samples = np.linspace(w[0]-w[1], w[0]+w[1], sampling)

        paths = []
        for vv in v_samples:
            for ww in w_samples:
                paths.append(self.simulate_path(v=vv, w=ww, T=t))

        return paths
    def point_msgs_to_np_array(self,points_msg_list):
        return np.array([[p.x, p.y] for p in points_msg_list])
    
    def cal_score(self,range_array,path_sim):
        path_np = np.array(path_sim[0].coords)
        for p in path_sim[1:]:
            p = np.array(p.coords)
            path_np = np.vstack([path_np,p])

        check = []
        score = []      
        for i,range in enumerate(self.sensor_range_g):
            print(i)
            # if self.points_in_polygon(path_np,range):
            #     check.append(i)

            # range_np = np.array(range.exterior.coords)[:-1] 
            # path_np_expanded = path_np[:, np.newaxis, :]    # (101, 1, 2)
            # range_np_expanded = range_np[np.newaxis, :, :]    # (1, 510000, 2)

            # # 차이 계산
            # diff = path_np_expanded - range_np_expanded  # (101, 510000, 2)

            # # 거리 계산
            # dists = np.linalg.norm(diff, axis=2)  # (101, 510000)

            # # 각 A[i]마다 평균
            # score.append(dists.mean())
        return check,score
        


def main(args=None):
    rclpy.init(args=args)
    node = FailSafeSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
