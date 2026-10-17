# Let's inspect media_1790259226818.png by finding all contours/lanes
from PIL import Image
import numpy as np

img = Image.open(r"C:\Users\rober\.gemini\antigravity\brain\77fea321-1c41-4f6d-8b80-8e02b6a94640\.user_uploaded\media_1790259226818.png").convert('RGB')
w, h = img.size
print(f"Image size: {w} x {h}")

# Let's crop the left loop (the big square on the left)
crop_left_loop = img.crop((0, 0, int(w * 0.55), h))
crop_left_loop.save(r"d:\nxpcup\scratch\crop_left_loop.png")
print("Saved crop_left_loop.png")
