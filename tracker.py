import cv2

point_selected = False
point = (0, 0)
tracker_initialized = False

url = 'http://192.168.10.132:8080/stream?topic=/main_camera/image_raw'

def select_point(event, x, y, flags, param):
    global point_selected, point, tracker_initialized
    if event == cv2.EVENT_LBUTTONDOWN:
        point_selected = True
        point = (x, y)
        tracker_initialized = False

cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Ошибка открытия камеры")
    exit()

cv2.namedWindow("Tracking")
cv2.setMouseCallback("Tracking", select_point)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if point_selected and not tracker_initialized:
        tracker = cv2.TrackerCSRT_create()
        bbox = (point[0] - 10, point[1] - 10, 20, 20)  # Начальный bounding box вокруг точки
        tracker.init(frame, bbox)
        tracker_initialized = True

    if tracker_initialized:
        success, bbox = tracker.update(frame)
        if success:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        else:
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

