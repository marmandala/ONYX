import cv2
import matplotlib.pyplot as plt
import os
import numpy as np

def visualize_yolo(image_path, txt_path, seg_txt_path, annotation_type):
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    height, width, _ = image.shape

    if annotation_type == 'detection':
        with open(txt_path, 'r') as file:
            lines = file.readlines()

        for line in lines:
            class_id, x_center, y_center, w, h = map(float, line.split())
            x_center *= width
            y_center *= height
            w *= width
            h *= height

            x1 = int(x_center - w / 2)
            y1 = int(y_center - h / 2)
            x2 = int(x_center + w / 2)
            y2 = int(y_center + h / 2)

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, str(int(class_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    elif annotation_type == 'segmentation' and seg_txt_path:
        with open(seg_txt_path, 'r') as file:
            lines = file.readlines()

        for line in lines:
            parts = line.split()
            class_id = int(parts[0])
            points = [float(p) for p in parts[1:]]

            polygon = []
            for i in range(0, len(points), 2):
                x = int(points[i] * width)
                y = int(points[i + 1] * height)
                polygon.append((x, y))

            polygon = np.array(polygon, dtype=np.int32)
            cv2.polylines(image, [polygon], isClosed=True, color=(255, 0, 0), thickness=2)
            cv2.putText(image, str(class_id), (polygon[0][0], polygon[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    return image


fig, ax = plt.subplots(figsize=(8, 6))

# Define the annotation type here: 'detection' for object detection, 'segmentation' for segmentation
annotation_type = 'segmentation'  # Change this to 'segmentation' as needed

for i in range(1):
    image_path_template = 'results/combined_mask_frame_17.png'
    txt_path_template = 'TXTs/combined_mask_frame_17.txt'
    seg_txt_path_template = 'TXTs/combined_mask_frame_17.txt'


    annotated_image = visualize_yolo(image_path_template, txt_path_template, seg_txt_path_template, annotation_type)

    ax.imshow(annotated_image)
    ax.axis('off')
    ax.set_title(f'G:/YOLO_UNITY/images/image_{i}.png')

    plt.pause(100000)

    ax.clear()

plt.close()
