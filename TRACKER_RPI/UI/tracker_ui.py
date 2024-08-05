from flask import Flask, render_template, Response, request, jsonify
import cv2
import requests
import numpy as np

app = Flask(__name__)

# Переменные для хранения координат
last_coordinates = None
box_coordinates = None

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
                    _, buffer = cv2.imencode('.jpg', img)
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            print("Failed to fetch the image.")
            break

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/box_coords', methods=['POST'])
def box_coords():
    global last_coordinates
    coordinates = request.form.get('coordinates')
    print(f'Published coordinates: {coordinates}')
    last_coordinates = coordinates
    return '', 204

@app.route('/box_coords', methods=['GET'])
def get_box_coords():
    if last_coordinates is not None:
        x, y = last_coordinates.split(',')
        return jsonify({'x': int(x), 'y': int(y)})
    else:
        return jsonify({'error': 'No coordinates available'}), 404

# Инициализация переменной для хранения полученных данных
received_data = None

@app.route('/post_pid_results', methods=['POST'])
def post_pid_results():
    global received_data
    try:
        data = request.json
        x = data.get('x')
        y = data.get('y')
        
        received_data = {'x': x, 'y': y}
        
        return 'Data received successfully', 200
    
    except Exception as e:
        return f'Error: {e}', 500

@app.route('/get_pid_results', methods=['GET'])
def get_pid_results():
    global received_data
    if received_data:
        return jsonify(received_data), 200
    else:
        return 'No data available', 404

if __name__ == '__main__':
    app.run(host="0.0.0.0", debug=True)

