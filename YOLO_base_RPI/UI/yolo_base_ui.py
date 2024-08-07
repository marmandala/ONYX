from flask import Flask, render_template, Response, request
import cv2
import requests
import numpy as np

app = Flask(__name__)

# Переменные для хранения маски
mask = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/upload_mask', methods=['POST'])
def upload_mask():
    global mask
    if 'mask' not in request.files:
        return "No mask file", 400

    mask_file = request.files['mask']
    npimg = np.frombuffer(mask_file.read(), np.uint8)
    mask = cv2.imdecode(npimg, cv2.IMREAD_GRAYSCALE)
    # Преобразование маски в двоичное изображение (0 или 255)
    _, mask = cv2.threshold(mask, 128, 255, cv2.THRESH_BINARY)
    return "Mask uploaded", 200

def get_frame():
    stream_url = "http://192.168.10.132:8080/stream?topic=/main_camera/image_raw"
    while True:
        img_resp = requests.get(stream_url, stream=True)
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
                    
                    # Apply the mask if it exists
                    if mask is not None:
                        # Resize mask to match the image size if necessary
                        if mask.shape != img.shape[:2]:
                            resized_mask = cv2.resize(mask, (img.shape[1], img.shape[0]))
                        else:
                            resized_mask = mask
                        
                        # Create color overlays
                        green_overlay = np.zeros_like(img)
                        red_overlay = np.zeros_like(img)
                        green_overlay[:] = (0, 255, 0)
                        red_overlay[:] = (0, 0, 255)
                        
                        # Apply mask to the overlays
                        green_part = cv2.bitwise_and(green_overlay, green_overlay, mask=resized_mask)
                        red_part = cv2.bitwise_and(red_overlay, red_overlay, mask=cv2.bitwise_not(resized_mask))
                        
                        # Combine overlays with the original image separately
                        img_with_green = cv2.addWeighted(img, 1.0, green_part, 0.5, 0)
                        img_with_red = cv2.addWeighted(img, 1.0, red_part, 0.5, 0)
                        img = cv2.addWeighted(img_with_green, 0.5, img_with_red, 0.5, 0)
                        
                        # Display the masks using cv2.imshow
                        # cv2.imshow('red_part', red_part)
                        # cv2.imshow('green_part', green_part)
                        # cv2.imshow('img_with_red', img_with_red)
                        # cv2.imshow('img_with_green', img_with_green)
                        # cv2.imshow('Inverse Mask', cv2.bitwise_not(resized_mask))
                        # cv2.imshow('Mask', resized_mask)
                        # cv2.waitKey(1)

                    _, buffer = cv2.imencode('.jpg', img)
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            print("Failed to fetch the image.")
            break

if __name__ == '__main__':
    app.run(host="0.0.0.0", debug=True)

