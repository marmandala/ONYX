import cv2
import numpy as np
import requests
import time

# Параметры PID контроллера
Kp = 0.01
Ki = 0
Kd = 0
integral_x = 0
prev_error_x = 0
integral_y = 0
prev_error_y = 0

url = 'http://192.168.10.132:8080/stream?topic=/main_camera/image_raw'

# Глобальные переменные для состояния трекера
point_selected = False
point = None
tracker_initialized = False
tracker = None
processing_interval = 0.1  # интервал между обработкой кадров в секундах
last_processed_time = 0

def fetch_image_from_stream(url):
    img_resp = requests.get(url, stream=True)
    if img_resp.status_code == 200:
        img_bytes = bytes()
        for chunk in img_resp.iter_content(chunk_size=1024):
            img_bytes += chunk
            a = img_bytes.find(b'\xff\xd8')
            b = img_bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = img_bytes[a:b+2]
                img_bytes = img_bytes[b+2:]
                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                return img
    else:
        print("Failed to fetch image.")
        return None

def initialize_tracker(initial_point):
    global point, tracker_initialized, tracker
    point = initial_point
    tracker = cv2.legacy.TrackerCSRT_create() if hasattr(cv2, 'legacy') else cv2.TrackerCSRT_create()
    frame = fetch_image_from_stream(url)
    if frame is None:
        print("Ошибка получения кадра для инициализации трекера")
        return False
    bbox = (max(0, point[0] - 10), max(0, point[1] - 10), 20, 20)
    tracker_initialized = tracker.init(frame, bbox)
    return tracker_initialized

def track_point_step():
    global integral_x, prev_error_x, integral_y, prev_error_y, last_processed_time, point, tracker_initialized, tracker

    frame = fetch_image_from_stream(url)
    if frame is None:
        print("Ошибка получения кадра")
        return False

    current_time = time.time()
    if current_time - last_processed_time < processing_interval:
        return True  # Продолжаем работу

    last_processed_time = current_time

    try:
        center_x = point[0]
        center_y = point[1]

        error_x = center_x - frame.shape[1] // 2
        error_y = center_y - frame.shape[0] // 2

        integral_x += error_x
        derivative_x = error_x - prev_error_x
        prev_error_x = error_x
        pid_output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x

        integral_y += error_y
        derivative_y = error_y - prev_error_y
        prev_error_y = error_y
        pid_output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

        # Отправка результатов PID-регулятора на другой адрес
        payload = {'x': pid_output_x, 'y': pid_output_y}
        response = requests.post('http://127.0.0.1:5000/post_pid_results', json=payload)
        if response.status_code != 200:
            print(f"Ошибка отправки результатов PID-регулятора: {response.text}")

        # Обновляем трекер
        if tracker_initialized:
            success, bbox = tracker.update(frame)
            if success:
                point = (int(bbox[0] + bbox[2] / 2), int(bbox[1] + bbox[3] / 2))
                payload = {'x': point[0], 'y': point[1]}
                response = requests.post('http://127.0.0.1:5000/box_coords', json=payload)
                cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])), (255, 0, 0), 2, 1)
            else:
                cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    except Exception as e:
        print(f"Ошибка обработки кадра: {e}")

    cv2.imshow('Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False

    return True

