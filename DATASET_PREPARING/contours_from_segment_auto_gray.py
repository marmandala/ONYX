import cv2
import numpy as np
import os


def read_grayscale_values_from_txt(txt_path):
    with open(txt_path, 'r') as f:
        line = f.readline().strip()
        grayscale_values = [int(value) for value in line.split(',')]
    return grayscale_values


def is_clockwise(contour):
    value = 0
    num = len(contour)
    for i, point in enumerate(contour):
        p1 = contour[i]
        if i < num - 1:
            p2 = contour[i + 1]
        else:
            p2 = contour[0]
        value += (p2[0][0] - p1[0][0]) * (p2[0][1] + p1[0][1])
    return value < 0


def get_merge_point_idx(contour1, contour2):
    idx1 = 0
    idx2 = 0
    distance_min = -1
    for i, p1 in enumerate(contour1):
        for j, p2 in enumerate(contour2):
            distance = pow(p2[0][0] - p1[0][0], 2) + pow(p2[0][1] - p1[0][1], 2)
            if distance_min < 0:
                distance_min = distance
                idx1 = i
                idx2 = j
            elif distance < distance_min:
                distance_min = distance
                idx1 = i
                idx2 = j
    return idx1, idx2


def merge_contours(contour1, contour2, idx1, idx2):
    contour = []
    for i in range(0, idx1 + 1):
        contour.append(contour1[i])
    for i in range(idx2, len(contour2)):
        contour.append(contour2[i])
    for i in range(0, idx2 + 1):
        contour.append(contour2[i])
    for i in range(idx1, len(contour1)):
        contour.append(contour1[i])
    contour = np.array(contour)
    return contour


def merge_with_parent(contour_parent, contour):
    if not is_clockwise(contour_parent):
        contour_parent = contour_parent[::-1]
    if is_clockwise(contour):
        contour = contour[::-1]
    idx1, idx2 = get_merge_point_idx(contour_parent, contour)
    return merge_contours(contour_parent, contour, idx1, idx2)


def mask2polygon(image):
    contours, hierarchies = cv2.findContours(image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
    contours_approx = []
    polygons = []
    for contour in contours:
        epsilon = 0.001 * cv2.arcLength(contour, True)
        contour_approx = cv2.approxPolyDP(contour, epsilon, True)
        contours_approx.append(contour_approx)

    contours_parent = []
    for i, contour in enumerate(contours_approx):
        parent_idx = hierarchies[0][i][3]
        if parent_idx < 0 and len(contour) >= 3:
            contours_parent.append(contour)
        else:
            contours_parent.append([])

    for i, contour in enumerate(contours_approx):
        parent_idx = hierarchies[0][i][3]
        if parent_idx >= 0 and len(contour) >= 3:
            contour_parent = contours_parent[parent_idx]
            if len(contour_parent) == 0:
                continue
            contours_parent[parent_idx] = merge_with_parent(contour_parent, contour)

    contours_parent_tmp = []
    for contour in contours_parent:
        if len(contour) == 0:
            continue
        contours_parent_tmp.append(contour)

    polygons = []
    for contour in contours_parent_tmp:
        polygon = contour.flatten().tolist()
        polygons.append(polygon)
    return polygons


def extract_contours(image_path, txt_path, buffer_size):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Ошибка при загрузке изображения {image_path}")
        return

    grayscale_values = read_grayscale_values_from_txt(txt_path)
    contours_dict = {}

    for grayscale_value in grayscale_values:
        mask = np.zeros_like(image)
        for i in range(grayscale_value - buffer_size, grayscale_value + buffer_size + 1):
            mask = np.bitwise_or(mask, np.uint8(image == i))

        polygons = mask2polygon(mask)
        contours_dict[int(grayscale_value)] = polygons

    return contours_dict


def save_contours_yolo(contours_dict, output_path, image_shape):
    height, width = image_shape

    yolo_lines = []
    for obj_id, contours in contours_dict.items():
        for contour in contours:
            normalized_contour = [(x / width, y / height) for x, y in zip(contour[::2], contour[1::2])]
            flattened_contour = [coord for point in normalized_contour for coord in point]
            yolo_line = f"0 " + " ".join(map(str, flattened_contour))
            yolo_lines.append(yolo_line)

    with open(output_path, 'w') as f:
        f.write("\n".join(yolo_lines))


def draw_and_show_contours(image_path, contours_dict, output_dir):
    image = cv2.imread(image_path)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for obj_id, contours in contours_dict.items():
        image_copy = image.copy()

        for contour in contours:
            contour_np = np.array(contour).reshape((-1, 1, 2))
            cv2.drawContours(image_copy, [contour_np], -1, (0, 255, 0), 2)

            for i in range(0, len(contour), 2):
                point = (int(contour[i]), int(contour[i + 1]))
                cv2.circle(image_copy, point, 3, (0, 0, 0), -1)

        output_image_path = os.path.join(output_dir, f"contours_obj_{obj_id}.png")

        cv2.imshow(f"Контуры объекта {obj_id}", image_copy)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def process_images(image_dir, txt_dir, output_contours_dir, output_yolo_dir, buffer_size):
    if not os.path.exists(output_contours_dir):
        os.makedirs(output_contours_dir)
    if not os.path.exists(output_yolo_dir):
        os.makedirs(output_yolo_dir)

    image_filenames = [f for f in os.listdir(image_dir) if f.endswith('.jpg') or f.endswith('.png')]
    for image_filename in image_filenames:
        image_path = os.path.join(image_dir, image_filename)
        txt_filename = os.path.splitext(image_filename)[0] + "_color.txt"
        txt_path = os.path.join(txt_dir, txt_filename)

        if not os.path.exists(txt_path):
            print(f"Текстовый файл {txt_path} не найден. Пропускаю {image_filename}.")
            continue

        contours_dict = extract_contours(image_path, txt_path, buffer_size)
        if contours_dict:
            image_shape = cv2.imread(image_path).shape[:2]
            output_yolo_path = os.path.join(output_yolo_dir, f"{os.path.splitext(image_filename)[0]}.txt")
            save_contours_yolo(contours_dict, output_yolo_path, image_shape)
            draw_and_show_contours(image_path, contours_dict, output_contours_dir)
            print(f"Контуры для {image_filename} сохранены в формате YOLO в файле {output_yolo_path}")
        else:
            print(f"Контуры не найдены для изображения {image_filename}")


if __name__ == "__main__":
    image_dir = "G:\\YOLO_PYTHON\\images"
    txt_dir = "G:\\YOLO_PYTHON\\output"
    output_contours_dir = "G:\\YOLO_PYTHON\\countours"
    output_yolo_dir = "G:\\YOLO_PYTHON\\TXTs"
    buffer_size = 1  # Измените это значение, чтобы настроить буфер

    process_images(image_dir, txt_dir, output_contours_dir, output_yolo_dir, buffer_size)
