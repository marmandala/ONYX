import os
from tkinter import Tk, Label, Button, filedialog
from PIL import Image, ImageTk
from natsort import natsorted
import keyboard

class ImageLabel(Label):
    def __init__(self, master, file_name_label, grayscale_label, **kwargs):
        Label.__init__(self, master, **kwargs)
        self.bind('<Button-1>', self.get_pixel_color)
        self.image = None
        self.image_path = ""
        self.app_instance = None
        self.file_name_label = file_name_label
        self.grayscale_label = grayscale_label
        self.grayscale_values = []
        self.check_and_display_grayscale()

    def set_image(self, image_path):
        self.image_path = image_path
        self.image = Image.open(image_path)
        self.photo = ImageTk.PhotoImage(self.image)
        self.config(image=self.photo)

    def set_app_instance(self, app_instance):
        self.app_instance = app_instance

    def get_pixel_color(self, event):
        if self.image:
            x, y = event.x, event.y
            rgb = self.image.getpixel((x, y))
            grayscale = int(0.3 * rgb[0] + 0.59 * rgb[1] + 0.11 * rgb[2])
            self.grayscale_values.append(grayscale)
            self.save_grayscale(self.grayscale_values)
            self.display_grayscale_values()

            if not keyboard.is_pressed('ctrl'):
                self.grayscale_values = []
                if self.app_instance:
                    self.app_instance.next_image()

    def save_grayscale(self, grayscale_values):
        base_path = "output"
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        image_name = os.path.basename(self.image_path)
        file_name, _ = os.path.splitext(image_name)
        txt_file_path = os.path.join(base_path, f"{file_name}_color.txt")

        with open(txt_file_path, "w") as f:
            for value in grayscale_values:
                f.write(f"{value}\n")

    def display_grayscale_values(self):
        grayscale_text = "Grayscale values: " + ", ".join(map(str, self.grayscale_values))
        self.grayscale_label.config(text=grayscale_text)

    def check_and_display_grayscale(self):
        if self.image_path:
            image_name = os.path.basename(self.image_path)
            file_name, _ = os.path.splitext(image_name)
            txt_file_path = os.path.join("output", f"{file_name}_color.txt")

            if os.path.exists(txt_file_path):
                with open(txt_file_path, "r") as f:
                    grayscale_values = [int(line.strip()) for line in f.readlines()]
                grayscale_text = "Grayscale values: " + ", ".join(map(str, grayscale_values))
            else:
                grayscale_text = "Grayscale values: None"

            self.grayscale_label.config(text=grayscale_text)
            self.file_name_label.config(text=image_name)


class ImageApp:
    def __init__(self, root, directory):
        self.root = root

        self.file_name_label = Label(root, text="", font=("Helvetica", 16))
        self.file_name_label.pack()

        self.grayscale_label = Label(root, text="", font=("Helvetica", 16))
        self.grayscale_label.pack()

        self.label = ImageLabel(root, self.file_name_label, self.grayscale_label)
        self.label.pack()

        self.next_button = Button(root, text="Next Image", command=self.next_image)
        self.next_button.pack()

        self.prev_button = Button(root, text="Previous Image", command=self.prev_image)
        self.prev_button.pack()

        self.directory = directory
        self.image_paths = self.load_images_from_directory()
        self.image_index = 0
        self.set_image()

        # Bind keyboard events to methods using 'keyboard' library
        keyboard.add_hotkey('right', self.next_image)
        keyboard.add_hotkey('left', self.prev_image)

    def load_images_from_directory(self):
        files = os.listdir(self.directory)
        image_extensions = ['.jpg', '.jpeg', '.png', '.gif', '.bmp', '.tiff']
        images = [file for file in files if os.path.splitext(file)[1].lower() in image_extensions]
        return natsorted([os.path.join(self.directory, image) for image in images])

    def set_image(self):
        if self.image_paths:
            image_path = self.image_paths[self.image_index]
            self.label.set_image(image_path)
            self.file_name_label.config(text=os.path.basename(image_path))
            self.label.check_and_display_grayscale()
            self.label.set_app_instance(self)

    def next_image(self):
        if self.image_paths:
            self.image_index = (self.image_index + 1) % len(self.image_paths)
            self.set_image()

    def prev_image(self):
        if self.image_paths:
            self.image_index = (self.image_index - 1) % len(self.image_paths)
            self.set_image()


if __name__ == "__main__":
    root = Tk()
    directory_path = filedialog.askdirectory(title="Select Directory with Images")
    if directory_path:
        app = ImageApp(root, directory_path)
        root.mainloop()

