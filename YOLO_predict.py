import cv2
import requests
import numpy as np
from ultralytics import YOLO

# Load the model
model = YOLO('landing_zone_seg.pt')

stream_url = "http://192.168.10.132:8080/stream?topic=/main_camera/image_raw"

def fetch_image_from_stream(url):
    try:
        img_resp = requests.get(url, stream=True, timeout=5)
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
    except Exception as e:
        print(f"Failed to fetch image: {e}")
    return None

while True:
    img = fetch_image_from_stream(stream_url)
    if img is not None:
        results = model(img, conf=0.99)
        
        # Annotated frame
        annotated_frame = results[0].plot()

        # Extract mask
        masks = results[0].masks
        if masks is not None:
            mask = masks.data[0]  # Assuming single mask, take the first one
            mask = mask.cpu().numpy()  # Convert to numpy array

            # Resize mask to match image dimensions
            mask = cv2.resize(mask, (img.shape[1], img.shape[0]))

            # Create a colored mask
            colored_mask = np.zeros_like(img)
            colored_mask[mask == 1] = [0, 255, 0]  # Green color mask

            # Overlay the mask on the original image
            masked_image = cv2.addWeighted(img, 1, colored_mask, 0.5, 0)

            # Display the images
            cv2.imshow('Annotated Frame', annotated_frame)
            cv2.imshow('Masked Image', masked_image)
        else:
            cv2.imshow('Annotated Frame', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Failed to fetch the image from the provided URL.")
        break

cv2.destroyAllWindows()

