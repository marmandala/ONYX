import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
import math

# Глобальные переменные
point_selected = False
point = (0, 0)
tracker_initialized = False

# Параметры PID контроллера
Kp = 0.5
Ki = 0
Kd = 0
integral_x = 0
prev_error_x = 0
integral_y = 0
prev_error_y = 0

rospy.init_node('flight')
bridge = CvBridge()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        raise Exception(res.message)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def select_point(event, x, y, flags, param):
    global point_selected, point, tracker_initialized
    if event == cv2.EVENT_LBUTTONDOWN:
        point_selected = True
        point = (x, y)
        tracker_initialized = False

# Открываем веб-камеру
cap = cv2.VideoCapture(0)

# Проверяем, открылась ли камера
if not cap.isOpened():
    print("Ошибка открытия камеры")
    exit()

# Создаем окно и устанавливаем функцию обратного вызова для мыши
cv2.namedWindow("Tracking")
cv2.setMouseCallback("Tracking", select_point)

navigate_wait(z=1.4, frame_id='body', auto_arm=True)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Инициализация трекера после клика пользователя
    if point_selected and not tracker_initialized:
        tracker = cv2.TrackerCSRT_create()
        bbox = (point[0] - 10, point[1] - 10, 20, 20)  # Начальный bounding box вокруг точки
        tracker.init(frame, bbox)
        tracker_initialized = True

    # Обновляем трекер
    if tracker_initialized:
        success, bbox = tracker.update(frame)
        if success:
            # Рисуем bounding box вокруг трекаемой точки
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)

            # PID control for drone movement
            center_x = int(bbox[0] + bbox[2] / 2)
            center_y = int(bbox[1] + bbox[3] / 2)
            image_center_x = frame.shape[1] // 2
            image_center_y = frame.shape[0] // 2

            error_x = center_x - image_center_x
            error_y = center_y - image_center_y

            integral_x += error_x
            derivative_x = error_x - prev_error_x
            prev_error_x = error_x
            pid_output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x

            integral_y += error_y
            derivative_y = error_y - prev_error_y
            prev_error_y = error_y
            pid_output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

            # Convert PID output to drone navigation commands
            navigate_wait(y=-pid_output_x / 100.0, x=-pid_output_y / 100.0, z=0, speed=0.5, frame_id='body')
        else:
            # В случае неудачи, отображаем текст ошибки
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Отображаем текущий кадр
    cv2.imshow("Tracking", frame)

    # Прерываем цикл по нажатию клавиши 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Освобождаем ресурсы
cap.release()
cv2.destroyAllWindows()
land_wait()

