import cv2
import requests
import numpy as np
from ultralytics import YOLO
import time

# Load the YOLO model
model = YOLO('landing_zone_seg.pt')

# Stream URL
stream_url = "http://192.168.10.132:8080/stream?topic=/main_camera/image_raw"
mask_endpoint = "http://192.168.10.132:5000/upload_mask"  # Replace with actual endpoint URL

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

def process_image(image):
    results = model(image, conf=0.7)
    
    # Assuming single mask, take the first one
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    if results[0].masks is not None:
        mask = results[0].masks.data[0].cpu().numpy()
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
        mask_colored = np.zeros_like(image)
        mask_colored[mask == 1] = (0, 255, 0)  # Green color for mask
    else:
        mask_colored = mask  # No mask, keep it black

    return image, mask_colored

def send_mask(mask, endpoint):
    _, buffer = cv2.imencode('.png', mask)
    response = requests.post(endpoint, files={'mask': buffer.tobytes()})
    if response.status_code == 200:
        print("Mask sent successfully.")
    else:
        print(f"Failed to send mask. Status code: {response.status_code}")

def main():
    start_time = time.time()

    while True:
        elapsed_time = time.time() - start_time
        print(f"Elapsed time: {elapsed_time:.2f} seconds")
        
        image = fetch_image_from_stream(stream_url)
        if image is None:
            print(f"Error: Failed to load image from stream {stream_url}")
            break

        processed_image, mask = process_image(image)
        send_mask(mask, mask_endpoint)
        
        # Overlay the mask on the original image
        overlay = cv2.addWeighted(processed_image, 0.7, mask, 0.3, 0)
        
        cv2.imshow('YOLOv8 Detection', overlay)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

