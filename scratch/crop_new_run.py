from PIL import Image

img = Image.open(r"C:\Users\rober\.gemini\antigravity\brain\77fea321-1c41-4f6d-8b80-8e02b6a94640\.user_uploaded\media_1790259226818.png")
w, h = img.size

# Crop top-left region:
crop1 = img.crop((int(w * 0.25), 0, int(w * 0.65), int(h * 0.6)))
crop1.save(r"d:\nxpcup\scratch\crop_ix_new.png")
print("Saved crop_ix_new.png")
