import os
import random
import shutil
import yaml


def create_yolo_datasets(image_dir, txt_dir, output_dir, train_percent=0.8, val_percent=0.1, test_percent=0.1):
    assert train_percent + val_percent + test_percent == 1.0, "Percentages must sum up to 1.0"

    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'train', 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'train', 'labels'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'val', 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'val', 'labels'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'test', 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'test', 'labels'), exist_ok=True)

    images = [f for f in os.listdir(image_dir) if f.endswith('.jpg') or f.endswith('.png')]

    random.shuffle(images)
    num_images = len(images)
    num_train = int(train_percent * num_images)
    num_val = int(val_percent * num_images)

    train_images = images[:num_train]
    val_images = images[num_train:num_train + num_val]
    test_images = images[num_train + num_val:]

    def copy_files(images, source_dir, dest_images_dir, dest_txt_dir):
        for img in images:
            img_name, img_ext = os.path.splitext(img)
            img_path = os.path.join(source_dir, img)
            txt_path = os.path.join(txt_dir, img_name + '.txt')
            shutil.copy(img_path, os.path.join(dest_images_dir, img))
            shutil.copy(txt_path, os.path.join(dest_txt_dir, img_name + '.txt'))

    copy_files(train_images, image_dir, os.path.join(output_dir, 'train', 'images'),
               os.path.join(output_dir, 'train', 'labels'))
    copy_files(val_images, image_dir, os.path.join(output_dir, 'val', 'images'),
               os.path.join(output_dir, 'val', 'labels'))
    copy_files(test_images, image_dir, os.path.join(output_dir, 'test', 'images'),
               os.path.join(output_dir, 'test', 'labels'))

    print(f"Total images: {num_images}")
    print(f"Train set: {len(train_images)} images")
    print(f"Validation set: {len(val_images)} images")
    print(f"Test set: {len(test_images)} images")

    data = {
        'train': './train',
        'val': './val',
        'test': './test',
        'nc': 1,
        'names': ['0']
    }

    yaml_file = os.path.join(output_dir, 'data.yaml')
    with open(yaml_file, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f"YAML файл создан: {yaml_file}")

dir = "G:\\YOLO_DIR\\"

image_dir = f'{dir}IMAGES'
txt_dir = f'{dir}LABELS'
output_dir = f'{dir}FOR_YOLO'

create_yolo_datasets(image_dir, txt_dir, output_dir)
