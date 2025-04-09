import rclpy
from rclpy.node import Node
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from scipy.spatial import cKDTree
import visualization_msgs.msg as visualization_msgs
import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class FailSafeSim(Node):
    def __init__(self):
        super().__init__('fail_safe_sim')

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
        self.sensor_range_g = []

        for i in range(self.sensor_num):
            self.sensor_publisher.append(
                self.create_publisher(geometry_msgs.PolygonStamped, f'sensor_range_{i}', 10)
            )
            self.sensor_fail_publisher.append(
                self.create_publisher(geometry_msgs.PolygonStamped, f'sensor_fail_{i}', 10)
            )
            x, y, theta, range_, fov = sensor_param[i]
            poly = self.create_sector(x, y, theta, range_, fov)
            self.sensor_range.append(self.to_poly_msg(poly))
            self.sensor_range_g.append(poly)

        self.path_pub = self.create_publisher(visualization_msgs.MarkerArray, 'paths', 10)
        self.subscriber_ = self.create_subscription(Int32, '/input_command', self.listener_callback, 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sensor_state = [True] * self.sensor_num
        self.vel = geometry_msgs.Twist()

        self.vel.linear.x = 0.
        self.vel.angular.z = 0.


        self.twist_sub = self.create_subscription(
            geometry_msgs.Twist,
            '/cmd_vel',  # 혹은 네가 원하는 토픽명
            self.twist_callback,
            10
        )


    def twist_callback(self, msg):
        self.vel = msg

    def listener_callback(self, msgs):
        self.sensor_state[msgs.data] = not self.sensor_state[msgs.data]

    def timer_callback(self):
        print("enbale",self.sensor_state)

        for i in range(self.sensor_num):
            if self.sensor_state[i]:
                self.sensor_publisher[i].publish(self.sensor_range[i])
            else:
                self.sensor_fail_publisher[i].publish(self.sensor_range[i])

        paths = self.path_sim(v=[self.vel.linear.x,0.2], w=[self.vel.angular.z,0.05])
        if paths is None:
            return
        msgs = visualization_msgs.MarkerArray()

        marker = visualization_msgs.Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lines"
        marker.id = i
        marker.action = visualization_msgs.Marker.DELETE
        msgs.markers.append(marker)

        for i, p in enumerate(paths):  # 너무 많으면 퍼블리시 느려지니까 일부만
            msgs.markers.append(self.to_path_msg(i, p))

        self.path_pub.publish(msgs)
        check, score = self.cal_score(paths)
        print('Active Sensors:', check)
        print('Scores:', score)

    def to_path_msg(self, id, line):
        marker = visualization_msgs.Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lines"
        marker.id = id
        marker.type = visualization_msgs.Marker.LINE_STRIP
        marker.action = visualization_msgs.Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        line_np = np.array(line.coords)
        for (x, y) in line_np:
            p = geometry_msgs.Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        return marker

    def to_poly_msg(self, poly):
        polygon_array = np.array(poly.exterior.coords)[:-1]
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
        return Polygon(points)

    def simulate_path(self, v, w, x0=0, y0=0, theta0=0, dt=0.1, T=10.0):
        x, y, theta = x0, y0, theta0
        positions = [(x, y)]

        for _ in range(int(T / dt)):
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += w * dt
            positions.append((x, y))

        return LineString(positions)

    def path_sim(self, v=[1.0, 0.2], w=[0.1, 0.05], t=5, sampling=30):
        v_center, v_error = v
        w_center, w_error = w
        if v_center <= v_error or w_center <= w_error:
            return None
        v_min = max(0.0, v_center - v_error)
        v_max = max(0.0, v_center + v_error)

        if v_min > v_max:
            v_min, v_max = v_max, v_min

        w_min = w_center - w_error
        w_max = w_center + w_error

        if w_min > w_max:
            w_min, w_max = w_max, w_min

        v_samples = np.linspace(v_min, v_max, sampling)
        w_samples = np.linspace(w_min, w_max, sampling)

        paths = []
        for vv in v_samples:
            for ww in w_samples:
                paths.append(self.simulate_path(v=vv, w=ww, T=t))

        return paths

    def cal_score(self, path_sim):
        # 모든 path를 하나로 합침
        path_np = np.vstack([np.array(p.coords) for p in path_sim])
        path_np = path_np[::5]  # subsample (선택사항)

        check = []
        score = []

        for i, poly in enumerate(self.sensor_range_g):
            inside = np.array([poly.contains(Point(pt)) for pt in path_np])

            if inside.any():
                check.append(i)

            distances = [poly.exterior.distance(Point(pt)) for pt in path_np]
            mean_distance = np.mean(distances)

            score.append(mean_distance)

        return check, score


def main(args=None):
    rclpy.init(args=args)
    node = FailSafeSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()