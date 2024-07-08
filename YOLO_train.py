import os
from ultralytics import YOLO

model = YOLO('yolov8n.pt')

model.train(data='/home/matvey/HTTP_TEST/gazebo cube.v1i.yolov8/data.yaml', epochs=25, imgsz=640)

model.save('yolov8_custom.pt')

