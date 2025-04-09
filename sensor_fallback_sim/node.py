import rclpy
from rclpy.node import Node
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from scipy.spatial import cKDTree
import visualization_msgs.msg as visualization_msgs
import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import pickle

class FailSafeSim(Node):
    def __init__(self):
        # ROS2 노드 초기화
        super().__init__('fail_safe_sim')

        # 각 센서의 위치(x,y), 방향(theta), 범위(range), 시야각(fov) 설정
        sensor_param = [ #총 15개
            # ZED 1 (전방)
            [1.2,  0.3,    0,   20, 100],
            # ZED 2 (후방)
            [-1.2, 0.3,  180,   20, 100],

            # OAK-D 1 (전방 좌)
            [1.0,  0.5,  -30,   10,  70],
            # OAK-D 2 (전방 우)
            [1.0, -0.5,   30,   10,  70],

            # 허사이 라이다(ATG128) - 루프 센터, 360도
            [0.0,  0.0,    0,  120, 360],

            # 반레어 라이다 1 (전방 좌)
            [1.2,  0.2,  -15,  60, 120],
            # 반레어 라이다 2 (전방 우)
            [1.2, -0.2,   15,  60, 120],


            # 반레어 레이더 1 (전방 좌 모서리)
            [1.2,  0.8,  -45,  150,  30],
            # 반레어 레이더 2 (전방 우 모서리)
            [1.2, -0.8,   45,  150,  30],
            # 반레어 레이더 3 (후방 좌 모서리)
            [-1.2, 0.8,  135,  150,  30],
            # 반레어 레이더 4 (후방 우 모서리)
            [-1.2, -0.8, -135, 150,  30],

            # IMU 1 (앞)
            [0.8,  0.0,    0,    0,   0],
            # IMU 2 (중앙)
            [0.0,  0.0,    0,    0,   0],
            # IMU 3 (뒤)
            [-0.8, 0.0,    0,    0,   0],

            # GNSS 1
            [0.0, -0.3,    0,    0,   0],
        ]

        self.sensor_num = len(sensor_param)  # 센서 개수
        self.sensor_publisher = []           # 센서가 정상일 때의 범위를 퍼블리시할 토픽 리스트
        self.sensor_fail_publisher = []      # 센서가 실패(비활성)했을 때의 범위를 퍼블리시할 토픽 리스트
        self.sensor_range = []               # 센서의 탐지 영역 메시지 저장
        self.sensor_range_g = []             # 센서 탐지 영역의 실제 Polygon 객체 저장

        self.sensor_R = [0.0]*self.sensor_num

        self.R_accum = 0.0 #누적되도록 R
        self.current_time = 0.0
        for i in range(self.sensor_num):
            # 센서 정상 상태에서 퍼블리시할 토픽 생성
            self.sensor_publisher.append(
                self.create_publisher(geometry_msgs.PolygonStamped, f'sensor_range_{i}', 10)
            )
            # 센서 실패 상태에서 퍼블리시할 토픽 생성
            self.sensor_fail_publisher.append(
                self.create_publisher(geometry_msgs.PolygonStamped, f'sensor_fail_{i}', 10)
            )

            x, y, theta, range_, fov = sensor_param[i]
            # 각 센서의 탐지 영역을 부채꼴 형태로 생성
            poly = self.create_sector(x, y, theta, range_, fov)
            # ROS 메시지 형태로 변환하여 저장
            self.sensor_range.append(self.to_poly_msg(poly))
            self.sensor_range_g.append(poly)

        # 경로(미래 경로 시뮬레이션) 시각화용 마커 퍼블리셔
        self.path_pub = self.create_publisher(visualization_msgs.MarkerArray, 'paths', 1)

        # 센서 상태를 변경하는 명령어 입력 구독
        self.subscriber_ = self.create_subscription(Int32, '/input_command', self.listener_callback, 10)

        # 주기적으로 실행될 타이머 (0.1초 간격)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 센서 활성화 상태(True=정상, False=비활성화)
        self.sensor_state = [True] * self.sensor_num

        # 로봇의 선속도, 각속도 초기화
        self.vel = geometry_msgs.Twist()
        self.vel.linear.x = 10.
        self.vel.angular.z = 0.3

        # cmd_vel (로봇 제어 명령)을 받아오는 구독자
        self.twist_sub = self.create_subscription(
            geometry_msgs.Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        self.lam = 0.01   # e^(lam*t)에서 lam
        self.dt = 0.1
        self.total_time_for_R = 5.0



    # 로봇 속도를 업데이트하는 콜백
    def twist_callback(self, msg):
        self.vel = msg

    # 센서 상태를 토글(활성화/비활성화)하는 명령 처리 콜백
    def listener_callback(self, msgs):
        self.sensor_state[msgs.data] = not self.sensor_state[msgs.data]

    # 주기적으로 실행되는 타이머 콜백
    def timer_callback(self):
        dt = 0.1
        self.current_time += dt
        print("enbale", self.sensor_state)

        # 센서 상태에 따라 정상/비정상 메시지 퍼블리시
        for i in range(self.sensor_num):
            if self.sensor_state[i]:
                self.sensor_publisher[i].publish(self.sensor_range[i])
                self.sensor_R[i] = 0.0
            else:
                self.sensor_fail_publisher[i].publish(self.sensor_range[i])
                self.sensor_R[i] += np.exp(self.lam * self.current_time) * dt 

        # 주어진 선속도 및 각속도 범위로 경로를 시뮬레이션하여 결과를 얻음
        paths = self.path_sim(v=[self.vel.linear.x, 0.2], w=[self.vel.angular.z, 0.05])
        if paths is None:
            return

        # 경로들을 MarkerArray 메시지로 변환하여 퍼블리시
        msgs = visualization_msgs.MarkerArray()

        # 이전 마커들을 삭제하는 메시지 추가
        marker = visualization_msgs.Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lines"
        marker.id = i
        marker.action = visualization_msgs.Marker.DELETE
        msgs.markers.append(marker)

        # 새 경로를 마커로 변환하여 추가
        for i, p in enumerate(paths):
            msgs.markers.append(self.to_path_msg(i, p))


        self.path_pub.publish(msgs)

        # 경로가 어떤 센서 영역에 들어가는지 체크, 거리 점수 계산하여 출력
        check, score = self.cal_score(paths)
        print('Active Sensors:', check)
        print('Scores:', score)

        sensor_order = sorted(range(self.sensor_num),key=lambda i:score[i]) # 스코어 기반 우선순위 
        print('Sensor Priority : ', sensor_order)
        k = 1.0
        sum_WL = 0.0

        # 센서 중 fail인 개수만큼 sum_WL += k
        for i in range(self.sensor_num):
            if not self.sensor_state[i]:  # fail 상태
                sum_WL += k


        self.current_time += dt
        R_total = sum(self.sensor_R)

        self.get_logger().info(f"R_total = {R_total}")


        data = {"t": self.current_time,
                "enable":self.sensor_state,
                "score": score,
                "check": check,
                "sensor_order":sensor_order,
                "R_total":R_total}

        with open("/home/taewook/ros2_ws/src/sensor_fallback_sim/sensor_fallback_sim.pkl", "ab") as f:  # append binary
            pickle.dump(data, f)



    # 시각화할 경로를 Marker 메시지로 변환하는 함수
    def to_path_msg(self, id, line):
        # Marker 메시지 설정
        marker = visualization_msgs.Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lines"
        marker.id = id
        marker.type = visualization_msgs.Marker.LINE_STRIP
        marker.action = visualization_msgs.Marker.ADD
        marker.scale.x = 1.0 
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        # 경로 좌표를 Marker 포인트로 추가
        line_np = np.array(line.coords)
        for (x, y) in line_np:
            p = geometry_msgs.Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        return marker

    # Shapely 폴리곤을 ROS 메시지로 변환하는 함수
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

    # 부채꼴 형태의 탐지영역을 생성하는 함수
    def create_sector(self, x, y, theta, radius, angle_deg, resolution=100):
        start_angle = theta - angle_deg / 2
        end_angle = theta + angle_deg / 2
        angles = np.radians(np.linspace(start_angle, end_angle, resolution))
        arc = [(x + radius * np.cos(a), y + radius * np.sin(a)) for a in angles]
        points = [(x, y)] + arc + [(x, y)]
        return Polygon(points)

# 나머지 메소드들은 위와 유사한 방식으로 경로 시뮬레이션 및 센서 평가를 수행


      # 로봇의 예상 경로를 시뮬레이션 하는 함수 (선속도 v, 각속도 w)
    def simulate_path(self, v, w, x0=0, y0=0, theta0=0, dt=0.1, T=10.0):
        x, y, theta = x0, y0, theta0  # 초기 위치 및 각도 설정
        positions = [(x, y)]          # 위치 기록할 리스트 생성

        for _ in range(int(T / dt)):  # 총 시간 T동안 dt 간격으로 반복
            # 로봇의 위치를 속도 기반으로 업데이트
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += w * dt
            positions.append((x, y))

        # 시뮬레이션한 경로를 LineString 형태로 반환
        return LineString(positions)

    # 다양한 속도 및 각속도로 여러 개의 경로를 시뮬레이션 하는 함수
    def path_sim(self, v=[1.0, 0.2], w=[0.1, 0.05], t=5, sampling=3):
        v_center, v_error = v
        w_center, w_error = w

        # # 잘못된 입력 예외 처리
        # if v_center <= v_error or w_center <= w_error:
        #     return None

        # 속도 및 각속도 범위를 정의
        v_min = max(0.0, v_center - v_error)
        v_max = v_center + v_error
        w_min = w_center - w_error
        w_max = w_center + w_error

        # 지정된 범위 내에서 여러 샘플을 추출하여 경로 시뮬레이션
        v_samples = np.linspace(v_min, v_max, sampling)
        w_samples = np.linspace(w_min, w_max, sampling)

        paths = []
        for vv in v_samples:
            for ww in w_samples:
                paths.append(self.simulate_path(v=vv, w=ww, T=t))

        return paths

    # 생성된 모든 경로를 평가하여 각 센서가 경로를 탐지 가능한지, 거리 점수를 계산하는 함수
    def cal_score(self, path_sim):
        # 모든 경로를 합쳐서 하나의 큰 배열로 생성
        path_np = np.vstack([np.array(p.coords) for p in path_sim])
        path_np = path_np[::5]  # 데이터 수를 줄이기 위해 subsample 적용

        check = []  # 활성화된(경로가 탐지 영역 안에 들어온) 센서의 인덱스
        score = []  # 각 센서별 거리 점수

        # 각 센서마다 경로 평가
        for i, poly in enumerate(self.sensor_range_g):
            # 경로의 좌표가 탐지 영역 안에 들어오는지 확인
            inside = np.array([poly.contains(Point(pt)) for pt in path_np])

            # 하나라도 안에 들어온 점이 있으면 활성 센서로 판단
            if inside.any():
                check.append(i)

            # 각 점이 센서의 탐지 영역 경계로부터 얼마나 떨어져 있는지 평균 계산
            distances = [poly.exterior.distance(Point(pt)) for pt in path_np]
            mean_distance = np.mean(distances)

            score.append(mean_distance)

        return check, score


def calc_R(self):
    lam = self.lam
    dt = self.dt_for_R
    t_max = self.total_time_for_R
    R_accum = 0.0
    t = 0.0

    while t <= t_max:
        sum_WL = 0.0  # 여기가 중요 (반드시 매루프마다 초기화)
        for i in range(self.sensor_num):
            for j in range(self.sensor_num):
                if i == j and not self.sensor_state[i]:
                    sum_WL += k  # W_ij * L_ij => k * 1

        R_accum += sum_WL * np.exp(lam * t)
        t += dt

    return R_accum

    
# ROS2 노드 실행 메인 함수
def main(args=None):
    rclpy.init(args=args)
    node = FailSafeSim()  # FailSafeSim 노드 생성
    rclpy.spin(node)      # ROS2 노드를 계속 실행
    node.destroy_node()   # 노드 종료 시 자원 해제
    rclpy.shutdown()      # ROS2 환경 종료

if __name__ == '__main__':
    main()
