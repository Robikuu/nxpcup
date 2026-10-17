from PIL import Image
import numpy as np

img = Image.open(r"d:\nxpcup\scratch\crop_ix_new.png").convert('RGB')
arr = np.array(img)
h, w, _ = arr.shape
print(f"Crop size: {w} x {h}")

# Find black lines (intensity < 50) and red lines (R > 180, G < 50, B < 50)
black = (arr[:, :, 0] < 80) & (arr[:, :, 1] < 80) & (arr[:, :, 2] < 80)
red = (arr[:, :, 0] > 180) & (arr[:, :, 1] < 60) & (arr[:, :, 2] < 60)

# Let's inspect columns from x=0 to w
# At x=20 (far left, the west arm): where are the black lines?
y_black_left = np.where(black[:, 20])[0]
print("Black pixels at x=20 (left west arm):", y_black_left)

# At x=w-30 (far right, top straight): where are the black lines?
y_black_right = np.where(black[:, w-30])[0]
print("Black pixels at x=w-30 (right top straight):", y_black_right)
