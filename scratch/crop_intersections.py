from PIL import Image

img = Image.open(r"C:\Users\rober\.gemini\antigravity\brain\77fea321-1c41-4f6d-8b80-8e02b6a94640\.user_uploaded\media_1790250468973.png")
w, h = img.size

# Look at the intersections:
# 1. Top intersection (around x=0.3 to 0.5, y=0.1 to 0.4)
inter_top = img.crop((w * 0.3, h * 0.1, w * 0.55, h * 0.4))
inter_top.save("scratch/inter_top.png")

# 2. Bottom intersection if any
inter_bot = img.crop((w * 0.3, h * 0.5, w * 0.6, h * 0.8))
inter_bot.save("scratch/inter_bot.png")

print("Saved inter_top and inter_bot")
