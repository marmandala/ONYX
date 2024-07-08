import requests

url = 'http://127.0.0.1:8000/post'
data = {
    'x': 4,
    'y': 5,
    'z': 1
}

response = requests.post(url, json=data)
if response.status_code == 201:
    print(response.json())
else:
    print('Произошла ошибка:', response.status_code)

