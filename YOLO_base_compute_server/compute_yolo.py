import cv2
import requests
import numpy as np
from ultralytics import YOLO
import time

# Загрузка модели YOLO
model = YOLO('best.pt')

# URL для потока
stream_url = "http://192.168.10.132:8080/stream?topic=/main_camera/image_raw"
mask_endpoint = "http://192.168.10.132:5000/upload_mask"  # Замените на фактический URL-адрес конечной точки

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
        print("Не удалось получить изображение.")
        return None

def process_image(image):
    results = model(image, conf=0.7)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)

    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, f'{model.names[int(box.cls[0])]} {box.conf[0]:.2f}', 
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)  # Заполняем обнаруженную область белым

    return image, mask

def send_mask(mask, endpoint):
    _, buffer = cv2.imencode('.png', mask)
    response = requests.post(endpoint, files={'mask': buffer.tobytes()})
    if response.status_code == 200:
        print("Маска успешно отправлена.")
    else:
        print(f"Не удалось отправить маску. Код состояния: {response.status_code}")

def main():
    start_time = time.time()

    while True:
        elapsed_time = time.time() - start_time
        print(f"Время работы кода: {elapsed_time:.2f} секунд")
        
        image = fetch_image_from_stream(stream_url)
        if image is None:
            print(f"Ошибка: не удалось загрузить изображение из потока {stream_url}")
            break

        processed_image, mask = process_image(image)
        send_mask(mask, mask_endpoint)
        
        cv2.imshow('YOLOv8 Detection', processed_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

