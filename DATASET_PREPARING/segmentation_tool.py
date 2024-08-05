import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk, ImageDraw
import cv2
import numpy as np
import os


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
    for i in list(range(0, idx1 + 1)):
        contour.append(contour1[i])
    for i in list(range(idx2, len(contour2))):
        contour.append(contour2[i])
    for i in list(range(0, idx2 + 1)):
        contour.append(contour2[i])
    for i in list(range(idx1, len(contour1))):
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
        if (parent_idx >= 0) and (len(contour) >= 3):
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


class YOLOSegmentationTool:
    def __init__(self, root):
        self.root = root
        self.root.title("YOLO Segmentation Tool")
        self.root.configure(bg="white")

        # Верхняя панель для изображения
        self.image_panel = tk.Label(root, bg="white")
        self.image_panel.pack(fill=tk.BOTH, expand=True)

        # Нижняя панель для кнопок и настройки кисти
        self.control_frame = tk.Frame(root, bg="white")
        self.control_frame.pack(fill=tk.X)

        self.load_button = tk.Button(self.control_frame, text="Load Image", command=self.load_image, bg="lightblue")
        self.load_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.save_button = tk.Button(self.control_frame, text="Save Annotations", command=self.save_annotations, bg="lightgreen")
        self.save_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.save_inverted_button = tk.Button(self.control_frame, text="Save Inverted Annotations", command=self.save_inverted_annotations, bg="lightcoral")
        self.save_inverted_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.brush_size_scale = tk.Scale(self.control_frame, from_=1, to=50, orient=tk.HORIZONTAL, label="Brush Size", bg="white")
        self.brush_size_scale.pack(side=tk.LEFT, padx=5, pady=5)

        self.canvas = None
        self.image = None
        self.image_path = ""
        self.mask_image = None
        self.start_x = None
        self.start_y = None

    def load_image(self):
        self.image_path = filedialog.askopenfilename()
        if self.image_path:
            self.image = cv2.cvtColor(cv2.imread(self.image_path), cv2.COLOR_BGR2RGB)
            self.mask_image = np.zeros(self.image.shape[:2], dtype=np.uint8)
            self.display_image()

    def display_image(self):
        if self.image is not None:
            if self.canvas:
                self.canvas.destroy()
            self.canvas = tk.Canvas(self.root, width=self.image.shape[1], height=self.image.shape[0])
            self.canvas.pack(fill=tk.BOTH, expand=True)
            self.photo_image = ImageTk.PhotoImage(image=Image.fromarray(self.image))
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo_image)
            self.canvas.bind("<B1-Motion>", self.paint)
            self.canvas.bind("<Motion>", self.show_brush_size)
            self.canvas.bind("<Leave>", self.hide_brush_size)

    def show_brush_size(self, event):
        self.hide_brush_size(event)
        brush_size = self.brush_size_scale.get()
        self.brush_indicator = self.canvas.create_oval(
            event.x - brush_size, event.y - brush_size,
            event.x + brush_size, event.y + brush_size,
            outline='red', width=2, stipple="gray25"
        )

    def hide_brush_size(self, event):
        if hasattr(self, 'brush_indicator'):
            self.canvas.delete(self.brush_indicator)

    def paint(self, event):
        brush_size = self.brush_size_scale.get()
        x1, y1 = (event.x - brush_size), (event.y - brush_size)
        x2, y2 = (event.x + brush_size), (event.y + brush_size)

        self.canvas.create_oval(x1, y1, x2, y2, fill="red", outline="red", stipple="gray50")
        overlay = self.mask_image.copy()
        cv2.circle(overlay, (event.x, event.y), brush_size, 255, -1)
        alpha = 0.4
        cv2.addWeighted(overlay, alpha, self.mask_image, 1 - alpha, 0, self.mask_image)

    def save_annotations(self):
        self.process_and_save_mask(self.mask_image, False)

    def save_inverted_annotations(self):
        inverted_mask = cv2.bitwise_not(self.mask_image)
        self.process_and_save_mask(inverted_mask, True)

    def process_and_save_mask(self, mask, is_inverted):
        polygons = mask2polygon(mask)
        img_h, img_w = self.image.shape[:2]
        annotations = []

        for polygon in polygons:
            annotation = "0"
            for i in range(0, len(polygon), 2):
                x = polygon[i] / img_w
                y = polygon[i + 1] / img_h
                annotation += f" {x} {y}"
            annotations.append(annotation)

        suffix = "_inverted" if is_inverted else ""
        annotation_path = os.path.splitext(self.image_path)[0] + f"{suffix}.txt"
        with open(annotation_path, "w") as f:
            f.write("\n".join(annotations))
        messagebox.showinfo("Success", f"Annotations saved to {annotation_path}")


if __name__ == "__main__":
    root = tk.Tk()
    app = YOLOSegmentationTool(root)
    root.mainloop()
