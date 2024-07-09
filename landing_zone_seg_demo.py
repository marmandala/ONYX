import requests
from PIL import Image
import torch
import torchvision.transforms as transforms

# Загрузка модели для сегментации (предполагается, что она уже обучена)
model_path = "landing_zone_seg.pt"  # Путь к вашей обученной модели
model = torch.load(model_path, map_location=torch.device('cpu'))
model.eval()

# URL для получения изображений с камеры
url = "http://192.168.10.132:8080/stream?topic=/main_camera/image_raw"

def process_image_from_url(url):
    # Получение изображения с URL
    response = requests.get(url, stream=True)
    image = Image.open(response.raw).convert('RGB')
    
    # Преобразование изображения для модели
    transform = transforms.Compose([
        transforms.Resize((512, 512)),  # Размер, соответствующий требованиям модели
        transforms.ToTensor(),
    ])
    image_tensor = transform(image).unsqueeze(0)
    
    # Применение модели к изображению
    with torch.no_grad():
        output = model(image_tensor)
    
    # Возвращение сегментированного изображения (здесь может быть необходимо преобразование)
    segmented_image = output.argmax(1).squeeze().cpu().numpy().astype('uint8')
    
    return segmented_image

# Обработка изображения и вывод результата
segmented_image = process_image_from_url(url)
# Далее можно добавить сохранение, отображение или другую обработку результата

