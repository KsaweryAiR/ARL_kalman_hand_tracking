import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mediapipe as mp
import cv2
import numpy as np
import time

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.velocity_publisher = self.create_publisher(Vector3, '/drone/velocity', 10)

        self.bridge = CvBridge()
        self.hands = mp.solutions.hands.Hands(max_num_hands=1)
        self.prev_time = time.time()

        self.kalmans = [Kalman_Filtering(1) for _ in range(21)]
        for kalman in self.kalmans:
            kalman.initialize()

        self.dron_x, self.dron_y = 260, 380

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        frame_height, frame_width, _ = frame.shape
        static_circle_x, static_circle_y = frame_width // 2, frame_height // 2

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]

            mp.solutions.drawing_utils.draw_landmarks(
                frame, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)

            thumb_tip = (int(hand_landmarks.landmark[4].x * frame.shape[1]), 
                        int(hand_landmarks.landmark[4].y * frame.shape[0]))
            index_tip = (int(hand_landmarks.landmark[8].x * frame.shape[1]), 
                        int(hand_landmarks.landmark[8].y * frame.shape[0]))

            cv2.circle(frame, (static_circle_x, static_circle_y), 10, (0, 255, 0), cv2.FILLED)

            if distance(thumb_tip, index_tip) < 25: 
                cv2.line(frame, (static_circle_x, static_circle_y), thumb_tip, (0, 255, 0), 3)

                points = np.array([[thumb_tip[0], thumb_tip[1]]], dtype=np.float32)
                kx, ky = self.kalmans[8].predict(points, dt)

                velocity = np.array([(kx - thumb_tip[0]) * 0.1, (ky - thumb_tip[1]) * 0.1], dtype=np.float32)

                velocity_msg = Vector3()
                velocity_msg.x = float(velocity[0])
                velocity_msg.y = float(velocity[1])
                velocity_msg.z = 0.0
                self.velocity_publisher.publish(velocity_msg)

                self.get_logger().info(f"Velocity: x={velocity[0]}, y={velocity[1]}")

        cv2.imshow("Drone Control", frame)
        cv2.waitKey(1)


def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

class Kalman_Filtering:
    def __init__(self, n_points):
        self.n_points = n_points

    def initialize(self):
        dt = 1
        n_states = self.n_points * 4
        n_measures = self.n_points * 2
        self.kalman = cv2.KalmanFilter(n_states, n_measures)
        self.kalman.transitionMatrix = np.eye(n_states, dtype=np.float32)
        self.kalman.measurementNoiseCov = np.eye(n_measures, dtype=np.float32) * 0.0005
        self.kalman.measurementMatrix = np.zeros((n_measures, n_states), np.float32)

        self.Measurement_array = []
        self.dt_array = []

        for i in range(0, n_states, 4):
            self.Measurement_array.append(i)
            self.Measurement_array.append(i + 1)

        for i in range(0, n_states):
            if i not in self.Measurement_array:
                self.dt_array.append(i)

        for i, j in zip(self.Measurement_array, self.dt_array):
            self.kalman.transitionMatrix[i, j] = dt

        for i in range(0, n_measures):
            self.kalman.measurementMatrix[i, self.Measurement_array[i]] = 1

    def predict(self, points, dt):
        for i, j in zip(self.Measurement_array, self.dt_array):
            self.kalman.transitionMatrix[i, j] = dt

        pred = []
        input_points = np.float32(np.ndarray.flatten(points))
        self.kalman.correct(input_points)
        tp = self.kalman.predict()

        for i in self.Measurement_array:
            pred.append(int(tp[i]))

        return pred

def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()  
    rclpy.shutdown()




#Stary kod, zostawiłem bo może być lepszy (bez kulki)

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Vector3
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import mediapipe as mp
# import cv2
# import numpy as np
# import time

# class DroneControlNode(Node):
#     def __init__(self):
#         super().__init__('drone_control_node')

#         self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
#         self.velocity_publisher = self.create_publisher(Vector3, '/drone/velocity', 10)

#         self.bridge = CvBridge()
#         self.hands = mp.solutions.hands.Hands(max_num_hands=1)
#         self.prev_time = time.time()

#         self.kalmans = [Kalman_Filtering(1) for _ in range(21)]
#         for kalman in self.kalmans:
#             kalman.initialize()

#         self.dron_x, self.dron_y = 260, 380

#     def listener_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         results = self.hands.process(frame_rgb)

#         current_time = time.time()
#         dt = current_time - self.prev_time
#         self.prev_time = current_time

#         if results.multi_hand_landmarks:
#             hand_landmarks = results.multi_hand_landmarks[0]

         
#             mp.solutions.drawing_utils.draw_landmarks(
#                 frame, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)

#             thumb_tip = (int(hand_landmarks.landmark[4].x * frame.shape[1]), 
#                          int(hand_landmarks.landmark[4].y * frame.shape[0]))
#             index_tip = (int(hand_landmarks.landmark[8].x * frame.shape[1]), 
#                          int(hand_landmarks.landmark[8].y * frame.shape[0]))

#             if distance(thumb_tip, index_tip) < 50:
#                 points = np.array([[thumb_tip[0], thumb_tip[1]]], dtype=np.float32)
#                 kx, ky = self.kalmans[8].predict(points, dt)
#                 velocity = np.array([(kx - thumb_tip[0]) * 0.1, (ky - thumb_tip[1]) * 0.1], dtype=np.float32)

#                 velocity_msg = Vector3()
#                 velocity_msg.x = float(velocity[0])
#                 velocity_msg.y = float(velocity[1])
#                 self.get_logger().info(f"Calculated velocity: x={velocity[0]}, y={velocity[1]}")

#                 velocity_msg.z = 0.0
#                 self.velocity_publisher.publish(velocity_msg)

        
#         cv2.imshow("Drone Control", frame)
#         cv2.waitKey(1)

# def distance(p1, p2):
#     return np.linalg.norm(np.array(p1) - np.array(p2))

# class Kalman_Filtering:
#     def __init__(self, n_points):
#         self.n_points = n_points

#     def initialize(self):
#         dt = 1
#         n_states = self.n_points * 4
#         n_measures = self.n_points * 2
#         self.kalman = cv2.KalmanFilter(n_states, n_measures)
#         self.kalman.transitionMatrix = np.eye(n_states, dtype=np.float32)
#         self.kalman.measurementNoiseCov = np.eye(n_measures, dtype=np.float32) * 0.0005
#         self.kalman.measurementMatrix = np.zeros((n_measures, n_states), np.float32)

#         self.Measurement_array = []
#         self.dt_array = []

#         for i in range(0, n_states, 4):
#             self.Measurement_array.append(i)
#             self.Measurement_array.append(i + 1)

#         for i in range(0, n_states):
#             if i not in self.Measurement_array:
#                 self.dt_array.append(i)

#         for i, j in zip(self.Measurement_array, self.dt_array):
#             self.kalman.transitionMatrix[i, j] = dt

#         for i in range(0, n_measures):
#             self.kalman.measurementMatrix[i, self.Measurement_array[i]] = 1

#     def predict(self, points, dt):
#         for i, j in zip(self.Measurement_array, self.dt_array):
#             self.kalman.transitionMatrix[i, j] = dt

#         pred = []
#         input_points = np.float32(np.ndarray.flatten(points))
#         self.kalman.correct(input_points)
#         tp = self.kalman.predict()

#         for i in self.Measurement_array:
#             pred.append(int(tp[i]))

#         return pred

# def main(args=None):
#     rclpy.init(args=args)
#     node = DroneControlNode()
#     rclpy.spin(node)
#     cv2.destroyAllWindows()  
#     rclpy.shutdown()

