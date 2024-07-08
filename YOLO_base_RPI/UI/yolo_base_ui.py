from flask import Flask, render_template, Response, request, jsonify
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
                        
                        # Apply mask to the image
                        img = cv2.bitwise_and(img, img, mask=resized_mask)

                    _, buffer = cv2.imencode('.jpg', img)
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            print("Failed to fetch the image.")
            break

if __name__ == '__main__':
    app.run(host="0.0.0.0", debug=True)

