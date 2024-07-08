import cv2
from ultralytics import YOLO

model = YOLO('best.pt')

image_path = 'test.jpg'
image = cv2.imread(image_path)

if image is None:
    print(f"Ошибка: не удалось загрузить изображение по пути {image_path}")
    exit(1)

results = model(image)

for box in results[0].boxes:
    x1, y1, x2, y2 = box.xyxy[0].tolist()
    confidence = box.conf[0]
    class_id = int(box.cls[0])
    
    cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    cv2.putText(image, f'{model.names[class_id]} {confidence:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

cv2.imshow('YOLOv8 Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

