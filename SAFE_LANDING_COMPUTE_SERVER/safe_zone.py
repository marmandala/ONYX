import cv2
import numpy as np
import requests
import compute_track_and_pid

def load_image_from_url(url):
    response = requests.get(url)
    image_data = np.frombuffer(response.content, np.uint8)
    image = cv2.imdecode(image_data, cv2.IMREAD_GRAYSCALE)
    return image

def find_landing_zone(mask, circle_diameter):
    h, w = mask.shape
    radius = circle_diameter // 2
    center = (w // 2, h // 2)
    best_point = None
    min_distance = float('inf')

    for y in range(radius, h - radius):
        for x in range(radius, w - radius):
            if mask[y, x] == 255:
                submask = mask[y - radius:y + radius, x - radius:x + radius]
                if np.all(submask == 255):
                    distance = np.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        best_point = (x, y)

    return best_point

def visualize_landing_zone(mask, landing_point, circle_diameter):
    output_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    if landing_point is not None:
        cv2.circle(output_image, landing_point, circle_diameter // 2, (0, 255, 0), 2)
    # cv2.imshow('Landing Zone', output_image)
    # cv2.waitKey(0)
    
    # if 0xFF == ord('q'):
        # cv2.destroyAllWindows()

# Пример использования
ip_address = 'http://192.168.10.132:5000/upload_mask'
circle_diameter = 50

# Загрузка маски с IP-адреса
mask = load_image_from_url(ip_address)

def recognize_landing_point():
    circle_diameter = 50
    mask = load_image_from_url(ip_address)
    
    landing_point = find_landing_zone(mask, circle_diameter)
    visualize_landing_zone(mask, landing_point, circle_diameter)
    return landing_point

# Визуализация
# visualize_landing_zone(mask, landing_point, circle_diameter)

# if landing_point:
    # print(f"Координаты безопасной зоны посадки: {landing_point}")
    # compute_track_and_pid.track_point(landing_point)
# else:
    # print("Безопасная зона для посадки не найдена.")

