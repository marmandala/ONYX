import os
from PIL import Image, ImageDraw
from natsort import natsorted
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from tkinter import filedialog, Tk
import keyboard


class ImageApp:
    def __init__(self, directory):
        self.directory = directory
        self.image_paths = self.load_images_from_directory()
        self.image_index = 0
        self.grayscale_recorded = False
        self.recorded_positions = []  # Список для хранения координат сохранённых грейскейлов

        self.fig, self.ax = plt.subplots(1, 2, figsize=(10, 5))
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

        self.next_button_ax = plt.axes([0.8, 0.01, 0.1, 0.05])
        self.prev_button_ax = plt.axes([0.1, 0.01, 0.1, 0.05])
        self.next_button = Button(self.next_button_ax, 'Next')
        self.prev_button = Button(self.prev_button_ax, 'Previous')

        self.next_button.on_clicked(self.next_image)
        self.prev_button.on_clicked(self.prev_image)

        self.grayscale_text = self.fig.text(0.5, 0.95, '', ha='center')

        self.set_image()

    def load_images_from_directory(self):
        files = os.listdir(self.directory)
        image_extensions = ['.jpg', '.jpeg', '.png', '.gif', '.bmp', '.tiff']
        images = [file for file in files if os.path.splitext(file)[1].lower() in image_extensions]
        return natsorted([os.path.join(self.directory, image) for image in images])

    def set_image(self):
        if self.image_paths:
            image_path = self.image_paths[self.image_index]
            self.image = Image.open(image_path)  # Сохранить текущее изображение
            self.grayscale_image = self.image.convert('L')

            self.ax[0].clear()
            self.ax[0].imshow(self.image)
            self.ax[0].set_title('Original Image')

            self.ax[1].clear()
            self.ax[1].imshow(self.grayscale_image, cmap='gray')
            self.ax[1].set_title('Grayscale Image')

            self.draw_recorded_positions()  # Нарисовать сохранённые позиции грейскейлов

            self.fig.suptitle(os.path.basename(image_path))

            self.update_grayscale_text()

            plt.draw()

    def update_grayscale_text(self):
        image_name = os.path.basename(self.image_paths[self.image_index])
        file_name, _ = os.path.splitext(image_name)
        txt_file_path = os.path.join("output", f"{file_name}_color.txt")

        if os.path.exists(txt_file_path):
            with open(txt_file_path, "r") as f:
                grayscale_values = f.read()
            grayscale_text = f"Grayscale values: {grayscale_values}"
        else:
            grayscale_text = "Grayscale values: None"

        if self.grayscale_text:
            self.grayscale_text.set_text(grayscale_text)
        else:
            self.grayscale_text = self.fig.text(0.5, 0.03, grayscale_text, ha='center',
                                                fontsize=self.fig._suptitle.get_fontsize())

        self.grayscale_text.set_fontsize(self.fig._suptitle.get_fontsize())
        self.grayscale_text.set_fontname(self.fig._suptitle.get_fontname())
        self.grayscale_text.set_position((0.5, 0.03))

        plt.draw()

    def draw_recorded_positions(self):
        draw = ImageDraw.Draw(self.image)
        for pos in self.recorded_positions:
            draw.ellipse((pos[0] - 5, pos[1] - 5, pos[0] + 5, pos[1] + 5), fill='red', outline='red')

    def next_image(self, event=None):
        if self.image_paths:
            self.image_index = (self.image_index + 1) % len(self.image_paths)
            self.set_image()

    def prev_image(self, event=None):
        if self.image_paths:
            self.image_index = (self.image_index - 1) % len(self.image_paths)
            self.set_image()

    def on_click(self, event):
        if event.inaxes == self.ax[0]:
            x, y = int(event.xdata), int(event.ydata)
            rgb = self.image.getpixel((x, y))
            grayscale = int(0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2])
            self.save_grayscale(x, y, grayscale)

            print(f"RGB: {rgb}, Grayscale: {grayscale}")
        elif event.inaxes == self.ax[1]:
            x, y = int(event.xdata), int(event.ydata)
            grayscale = self.grayscale_image.getpixel((x, y))
            self.save_grayscale(x, y, grayscale)

            print(f"Grayscale: {grayscale}")

    def save_grayscale(self, x, y, grayscale):
        grayscale = grayscale
        base_path = "output"
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        image_name = os.path.basename(self.image_paths[self.image_index])
        file_name, _ = os.path.splitext(image_name)
        txt_file_path = os.path.join(base_path, f"{file_name}_color.txt")

        if keyboard.is_pressed('ctrl'):
            if os.path.exists(txt_file_path):
                with open(txt_file_path, "r") as f:
                    existing_data = f.read().strip()

                # Check if grayscale value already exists in the file
                if not existing_data or str(grayscale) not in existing_data.split(','):
                    with open(txt_file_path, "a") as f:
                        if existing_data:
                            f.write(f",{grayscale}")
                        else:
                            f.write(str(grayscale))
            else:
                with open(txt_file_path, "w") as f:
                    f.write(str(grayscale))

            # Добавить координаты в список сохранённых позиций
            self.recorded_positions.append((x, y))
        else:
            with open(txt_file_path, "w") as f:
                f.write(str(grayscale))

        self.grayscale_recorded = True  # Always set to True after saving grayscale
        self.update_grayscale_text()

    def on_key_release(self, event):
        print(f"Key released: {event.key}")
        if event.key == 'control' and self.grayscale_recorded:
            self.grayscale_recorded = False
            self.next_image()

    def on_key_press(self, event):
        if event.key == 'right':
            self.next_image()
        elif event.key == 'left':
            self.prev_image()


if __name__ == "__main__":
    root = Tk()
    root.withdraw()  # Скрыть основное окно Tkinter
    directory_path = filedialog.askdirectory(title="Выберите директорию с изображениями")
    root.destroy()  # Закрыть основное окно Tkinter

    if directory_path:
        app = ImageApp(directory_path)
        plt.show()

