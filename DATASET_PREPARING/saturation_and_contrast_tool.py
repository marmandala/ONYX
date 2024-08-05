import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageEnhance, ImageTk


def open_image():
    global img, img_display
    file_path = filedialog.askopenfilename()
    if file_path:
        img = Image.open(file_path)
        update_image()


def save_image():
    if img:
        file_path = filedialog.asksaveasfilename(defaultextension=".jpg",
                                                 filetypes=[("JPEG files", "*.jpg"), ("All files", "*.*")])
        if file_path:
            edited_img.save(file_path)


def update_image():
    global edited_img, img_display
    if img:
        saturation = saturation_scale.get()
        contrast = contrast_scale.get()

        edited_img = ImageEnhance.Color(img).enhance(saturation)
        edited_img = ImageEnhance.Contrast(edited_img).enhance(contrast)

        img_display = ImageTk.PhotoImage(edited_img)
        canvas.itemconfig(image_container, image=img_display)

root = tk.Tk()
root.title("Photo Editor")

canvas = tk.Canvas(root, width=600, height=400)
canvas.pack()

btn_frame = tk.Frame(root)
btn_frame.pack()

open_btn = tk.Button(btn_frame, text="Open Image", command=open_image)
open_btn.pack(side=tk.LEFT, padx=5, pady=5)

save_btn = tk.Button(btn_frame, text="Save Image", command=save_image)
save_btn.pack(side=tk.LEFT, padx=5, pady=5)

slider_frame = tk.Frame(root)
slider_frame.pack()

tk.Label(slider_frame, text="Saturation").pack(side=tk.LEFT, padx=5, pady=5)
saturation_scale = tk.Scale(slider_frame, from_=0.01, to_=20, resolution=0.1, orient=tk.HORIZONTAL,
                            command=lambda x: update_image())
saturation_scale.set(1)
saturation_scale.pack(side=tk.LEFT, padx=5, pady=5)

tk.Label(slider_frame, text="Contrast").pack(side=tk.LEFT, padx=5, pady=5)
contrast_scale = tk.Scale(slider_frame, from_=0.01, to_=20, resolution=0.1, orient=tk.HORIZONTAL,
                          command=lambda x: update_image())
contrast_scale.set(1)
contrast_scale.pack(side=tk.LEFT, padx=5, pady=5)

img = None
edited_img = None
img_display = None

image_container = canvas.create_image(300, 200, anchor=tk.CENTER)

root.mainloop()
