from PIL import Image

img = Image.open(r"C:\Users\rober\.gemini\antigravity\brain\77fea321-1c41-4f6d-8b80-8e02b6a94640\.user_uploaded\media_1790257947704.png")
w, h = img.size
print(f"Image dimensions: {w} x {h}")

# Top region crop
top_crop = img.crop((int(w * 0.35), 0, int(w * 0.8), int(h * 0.5)))
top_crop.save(r"d:\nxpcup\scratch\crop_top.png")

# Full layout crop
print("Saved crop_top.png")
