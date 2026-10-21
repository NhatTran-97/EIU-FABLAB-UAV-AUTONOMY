"""Add a soft white halo around the drone icon (contrast on satellite imagery).
Baked into the PNG instead of a QML MultiEffect, which needs GPU shaders that are not available everywhere.
    python3 add_halo.py <drone_top.png>      (overwrites the file)"""
import sys

from PIL import Image, ImageFilter

path = sys.argv[1]
img = Image.open(path).convert('RGBA')
alpha = img.getchannel('A')
grow = alpha.filter(ImageFilter.MaxFilter(7)).filter(ImageFilter.GaussianBlur(2.5))
halo = Image.new('RGBA', img.size, (255, 255, 255, 0))
halo.putalpha(grow.point(lambda a: int(min(255, a * 0.85))))
Image.alpha_composite(halo, img).save(path)
print('halo ->', path)
