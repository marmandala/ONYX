import math
import rospy
import os
import time
from datetime import datetime
import cv2
from clover import srv
from std_srvs.srv import Trigger
import threading

# rospy.init_node('flight')  # Убедитесь, что инициализация узла происходит только один раз, если необходимо

square_size = 4
gap = 1
speed = 1

url = 'http://localhost:8080/stream?topic=/main_camera/image_raw'
cap = cv2.VideoCapture(url)

output_dir = 'saved_frames'
os.makedirs(output_dir, exist_ok=True)

start_time = time.time()
interval = 0.4
start_time_lock = threading.Lock()

if not cap.isOpened():
    print("Ошибка: Не удалось открыть поток видео")
    exit()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land_srv = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=speed, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

def capture():
    global start_time
    i = 0
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            print("Ошибка: Не удалось получить кадр")
            break

        cv2.imshow('IP Camera Stream', frame)

        current_time = time.time()
        with start_time_lock:
            if current_time - start_time >= interval:
                start_time = current_time
                filename = os.path.join(output_dir, f'frame_{i}.jpg')
                i = i + 1
                cv2.imwrite(filename, frame)
                print(f'Сохранен кадр: {filename}')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def scan_area():
    navigate_wait(x=0, y=0, z=3, speed=speed, frame_id='body', auto_arm=True)
    
    for i in range(int(square_size / gap / 2)):
        navigate_wait(x=0, y=square_size, z=0, frame_id='body', auto_arm=True)
        navigate_wait(x=gap, y=0, z=0, frame_id='body', auto_arm=True)
        navigate_wait(x=0, y=-square_size, z=0, frame_id='body', auto_arm=True)
        if i < ((square_size // gap // 2) - 1):
            print(i, (square_size // gap // 2) - 1)
            navigate_wait(x=gap, y=0, z=0, frame_id='body', auto_arm=True)

    land_srv()

def main():
    rospy.init_node('flight')
    
    capture_thread = threading.Thread(target=capture)
    scan_thread = threading.Thread(target=scan_area)
    
    scan_thread.start()
    capture_thread.start()

    #scan_thread.join()

if __name__ == "__main__":
    main()

