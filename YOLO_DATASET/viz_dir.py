import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.widgets import Button

# Function to visualize YOLO annotations on the image
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

# Directory containing images and annotations
image_dir = 'results'
txt_dir = 'TXTs'

# List all image files and corresponding annotation files
image_files = sorted([f for f in os.listdir(image_dir) if f.endswith('.png')])
txt_files = sorted([f for f in os.listdir(txt_dir) if f.endswith('.txt')])

# Initialize the current index
current_index = 0

# Define the annotation type here: 'detection' for object detection, 'segmentation' for segmentation
annotation_type = 'segmentation'  # Change this to 'detection' as needed

# Function to update the displayed image
def update_image(index):
    ax.clear()
    image_path = os.path.join(image_dir, image_files[index])
    txt_path = os.path.join(txt_dir, txt_files[index])
    annotated_image = visualize_yolo(image_path, txt_path, txt_path, annotation_type)
    ax.imshow(annotated_image)
    ax.axis('off')
    plt.draw()

# Button click event handlers
def next_image(event):
    global current_index
    current_index = (current_index + 1) % len(image_files)
    update_image(current_index)

def prev_image(event):
    global current_index
    current_index = (current_index - 1) % len(image_files)
    update_image(current_index)

# Set up the plot and buttons
fig, ax = plt.subplots(figsize=(8, 6))
axprev = plt.axes([0.7, 0.05, 0.1, 0.075])
axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
bnext = Button(axnext, 'Next')
bnext.on_clicked(next_image)
bprev = Button(axprev, 'Previous')
bprev.on_clicked(prev_image)

# Display the first image
update_image(current_index)
plt.show()

