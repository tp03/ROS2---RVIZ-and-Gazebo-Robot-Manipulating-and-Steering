## Treść zadania: **Wykorzystując Simple Commander API1 napisać węzeł ROS2, który zrealizuje ruch przez listę zadanych punktów, a podczas ruchu obrotowego węzeł dodatkowo steruje głową robota obracając ją w kierunku zgodnym z ruchem obrotowym bazy mobilnej.**

Zgodnie z treścią zadania stworzono nowy węzeł o nazwie *projekt2* którego zadaniem jest poruszenie się po zadanych punktach z głową robota poruszającą się w odpowiednim kierunku.
Funkcja main wywołująca nasz węzeł wygląda następująco.
```python
def main(args=None):
    rclpy.init(args=args)

    node = projekt2()

    points = [
        (-3.65, -0.844, -0.26),
        (4.13, 0.52, -1.67),
        (-2.56, -3.8, 1.42),
        (2.35, 4.8, -1.595),
    ]

    try:
        time.sleep(3)
        node.send_server_goal(points)
        rclpy.spin(node)        
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```
Punkty zadawane są do metody *send_server_goal*, gdzie każdy z punktów to krotka z trzema współrzędnymi x,y oraz z.
Nasz węzeł komunikuje się z kilkoma innymi węzłami, co zostało zaprezentowane w poniższym listingu z inicjalizacji naszego węzła.
```python
def __init__(self):
        super().__init__('projekt2')

        self.basic_navigator = BasicNavigator()

        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.velocity_subscriber = self.create_subscription(Twist, '/cmd_vel_nav', self.velocity_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.action_server = ActionServer(self, NavigatePoints, 'navigate_points', self.server_callback)
        self.action_client = ActionClient(self, NavigatePoints, 'navigate_points')

        self.current_velocity = 0.0

        self.previous_pose = []
        self.current_pose = [] 


```
Po pierwsze węzeł publikuje informacje typu JointTrajectory na temat /head_controller/joint_trajectory w celu kontrolowania ruchu głową. Wykonywane to jest w metodzie *move_head* odpowiedzialnej za ruch głowy pokazanej poniżej.
``` python
def move_head(self, head_1_angle, time_from_start=1):

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        point.positions = [head_1_angle, 0.0]
        point.time_from_start.sec = int(time_from_start)

        trajectory_msg.points = [point]
        self.head_publisher.publish(trajectory_msg)

```
Jak widać w powyższym kodzie jedynie wpływamy na jeden ze stawów głowy robota, ponieważ nie ma potrzeby porusznia nią góra-dół.

Nasz węzeł subskrybuje na temacie cmd_vel_nav, który zwraca informacje o prędkości robota. W przypadku nowej wiadomości na tym temacie, to znaczy zmiany prędkości, wywoływana jest funkcja self.velocity_callback która jest pokazana poniżej.
```python
def velocity_callback(self, msg):

        self.current_velocity = msg.angular.z
        if abs(self.current_velocity) < 0.01:
            self.move_head(0.0)
        else:
            angle = self.current_velocity*math.pi/3
            self.move_head(angle)

```
Jak widać w powyższej funkcji. Nasz robot w przypadku minimalnej lub zerowej prędkości ustawia głowę na wprost. ???



self.velocity_subscriber = self.create_subscription(Twist, '/cmd_vel_nav', self.velocity_callback, 10)
Węzeł subskrybuje wiadomości typu Twist na temat /cmd_vel_nav, które zawierają dane o prędkości robota. Funkcja self.velocity_callback jest wywoływana, gdy przyjdzie nowa wiadomość na tym temacie.

self.odom_subscriber = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
Węzeł subskrybuje wiadomości typu Odometry na temat /mobile_base_controller/odom, które zawierają dane o położeniu i prędkości robota w przestrzeni. Funkcja self.odom_callback jest wywoływana, gdy przyjdą nowe dane o odometrii.

Serwery i Klienci Akcji:

self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
Węzeł jest klientem akcji, który wysyła żądania do serwera akcji follow_waypoints. Serwer ten prawdopodobnie obsługuje zadanie podążania za punktami nawigacyjnymi.

self.action_server = ActionServer(self, NavigatePoints, 'navigate_points', self.server_callback)
Węzeł działa jako serwer akcji, który obsługuje żądania nawigacyjne związane z tematem navigate_points. Funkcja self.server_callback jest wywoływana, aby obsłużyć te żądania.

self.action_client = ActionClient(self, NavigatePoints, 'navigate_points')
Ten węzeł jest również klientem akcji dla tematu navigate_points, co oznacza, że może wysyłać żądania nawigacyjne do serwera akcji.