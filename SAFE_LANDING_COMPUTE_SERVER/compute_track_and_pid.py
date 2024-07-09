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

def track_point(initial_point):
    global integral_x, prev_error_x, integral_y, prev_error_y
    
    # Открываем веб-камеру
    cap = cv2.VideoCapture(url)

    # Проверяем, открылась ли камера
    if not cap.isOpened():
        print("Ошибка открытия камеры")
        return

    point_selected = False
    point = initial_point
    tracker_initialized = False
    tracker = None

    # Параметр для регулировки FPS
    processing_interval = 0.3  # интервал между обработкой кадров в секундах
    last_processed_time = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка получения кадра")
            break
        print("Кадр получен")

        current_time = time.time()
        if current_time - last_processed_time < processing_interval:
            # Пропускаем обработку этого кадра
            continue

        last_processed_time = current_time

        # Инициализация трекера
        if not tracker_initialized:
            tracker = cv2.legacy.TrackerCSRT_create() if hasattr(cv2, 'legacy') else cv2.TrackerCSRT_create()
            bbox = (max(0, point[0] - 10), max(0, point[1] - 10), 20, 20)
            tracker_initialized = tracker.init(frame, bbox)
            if tracker_initialized:
                print(f"Трекер инициализирован с bbox: {bbox}")
            else:
                print("Ошибка инициализации трекера")
                tracker = None

        try:
            center_x = point[0]
            center_y = point[1]
            print(f"Центр: {center_x}, {center_y}")

            error_x = center_x - frame.shape[1] // 2
            error_y = center_y - frame.shape[0] // 2
            print(f"Ошибки: error_x = {error_x}, error_y = {error_y}")

            integral_x += error_x
            derivative_x = error_x - prev_error_x
            prev_error_x = error_x
            pid_output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x
            print(f"PID X: {pid_output_x}")

            integral_y += error_y
            derivative_y = error_y - prev_error_y
            prev_error_y = error_y
            pid_output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y
            print(f"PID Y: {pid_output_y}")

            # Отправка результатов PID-регулятора на другой адрес
            payload = {'x': pid_output_x, 'y': pid_output_y}
            response = requests.post('http://127.0.0.1:5000/post_pid_results', json=payload)
            print(f"Отправка PID результатов: {payload}")
            if response.status_code != 200:
                print(f"Ошибка отправки результатов PID-регулятора: {response.text}")

            # Обновляем трекер
            if tracker_initialized:
                success, bbox = tracker.update(frame)
                if success:
                    point = (int(bbox[0] + bbox[2] / 2), int(bbox[1] + bbox[3] / 2))
                    payload = {'x': point[0], 'y': point[1]}
                    response = requests.post('http://127.0.0.1:5000/box_coords', json=payload)
                    print(f"Отправка координат центра трекера: {payload}")
                    # Рисуем bounding box вокруг трекаемой точки
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                else:
                    # В случае неудачи, отображаем текст ошибки
                    cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                    print("Ошибка отслеживания")

        except Exception as e:
            print(f"Ошибка обработки кадра: {e}")

        # Отображаем кадр с аннотациями в окне OpenCV
        cv2.imshow('Tracking', frame)

        # Прерываем цикл по нажатию клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Пример вызова функции с начальной точкой
# initial_point = (100, 100)
# track_point(initial_point)

