from PIL import Image

img = Image.open(r"C:\Users\rober\.gemini\antigravity\brain\77fea321-1c41-4f6d-8b80-8e02b6a94640\.user_uploaded\media_1790250468973.png")
w, h = img.size

# Bottom crop
crop_bot = img.crop((0, h * 0.4, w, h))
crop_bot.save("scratch/crop_bot.png")

# Left crop
crop_left = img.crop((0, 0, w * 0.5, h))
crop_left.save("scratch/crop_left.png")

print("Saved bottom and left crops")
