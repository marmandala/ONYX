import cv2

url = 'http://localhost:8080/stream?topic=/main_camera/image_raw'

cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Ошибка: Не удалось открыть поток видео")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Ошибка: Не удалось получить кадр")
        break

    cv2.imshow('IP Camera Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

