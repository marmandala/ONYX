from flask import Flask, request, jsonify
import flight

app = Flask(__name__)

@app.route('/test', methods=['GET'])
def give_sample_data():
    sample_data = {
        'id': 1,
        'title': 'Sample Data',
        'body': 'This is a sample data'
    }
    return jsonify(sample_data)

@app.route('/post', methods=['POST'])
def recieve_data():
    data = request.get_json()

    x = data.get('x')
    y = data.get('y')
    z = data.get('z')

    print(f"x: {x}, y: {y}, z: {z}")

    # Запуск полетной функции
    flight_operation(x, y, z)

    return jsonify({'x': x, 'y': y, 'z': z}), 201

def flight_operation(x, y, z):
    if x is not None and y is not None and z is not None:
        flight.takeoff()
        flight.navigate_to(x, y, z)
        flight.land()

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8000)

