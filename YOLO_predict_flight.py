import cv2
import torch
import numpy as np
from ultralytics import YOLO

# Load the model and move it to the GPU
model = YOLO('last.pt').to('cuda')

url = 'http://192.168.10.132:8080/stream?topic=/main_camera/image_raw'

while True:
    cap = cv2.VideoCapture(url)
    
    if not cap.isOpened():
        print("Ошибка: Не удалось открыть поток видео")
        exit()
    
    ret, frame = cap.read()
    if not ret:
        print("Ошибка: Не удалось получить кадр")
        break

    # Resize the frame to a size divisible by 32
    frame_resized = cv2.resize(frame, (640, 480))

    # Convert frame to tensor, add batch dimension, and move to GPU
    frame_tensor = torch.from_numpy(frame_resized).permute(2, 0, 1).unsqueeze(0).float().cuda() / 255.0

    # Perform inference on the GPU
    results = model(frame_tensor)

    # If masks are available, process them
    if results[0].masks is not None:
        for mask in results[0].masks.data:
            mask_np = mask.cpu().numpy()
            # Ensure the mask is a 2D array before resizing
            if len(mask_np.shape) > 2:
                mask_np = mask_np[0]
            mask_resized = cv2.resize(mask_np, (640, 480))

            color = (0, 255, 0)  # Green color for the mask
            # Create a 3-channel mask for blending
            mask_3ch = np.stack((mask_resized,) * 3, axis=-1)
            frame_resized = cv2.addWeighted(frame_resized, 1, mask_3ch.astype(frame_resized.dtype) * color, 0.5, 0)

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            confidence = box.conf[0]
            class_id = int(box.cls[0])

            cv2.rectangle(frame_resized, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame_resized, f'{model.names[class_id]} {confidence:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('IP Camera Stream', frame_resized)
    print("WORK")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

