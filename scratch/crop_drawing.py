from PIL import Image

img = Image.open(r"C:\Users\rober\.gemini\antigravity\brain\77fea321-1c41-4f6d-8b80-8e02b6a94640\.user_uploaded\media_1790250468973.png")
w, h = img.size
print(f"Image size: {w}x{h}")

# Save cropped regions:
# 1. Top intersection / entry
crop_top = img.crop((w * 0.2, 0, w * 0.8, h * 0.5))
crop_top.save("scratch/crop_top.png")

# 2. Middle loop ("ring around the rosie")
crop_mid = img.crop((w * 0.35, h * 0.15, w * 0.85, h * 0.85))
crop_mid.save("scratch/crop_mid.png")

# 3. Full track overview
print("Crops saved successfully")
