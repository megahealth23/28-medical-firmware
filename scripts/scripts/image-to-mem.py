#!/usr/bin/env python

# vim: set ai et ts=4 sw=4:

from PIL import Image
import sys
import os

if len(sys.argv) < 2:
    print("Usage: {} <image-file>".format(sys.argv[0]))
    sys.exit(1)

fname = sys.argv[1]

img = Image.open(fname)
print(img.width, img.height)
# if img.width != 160 or img.height != 80:
#     print("Error: 128x128 image expected")
#     sys.exit(2)

f = open("code.c", 'w')
# print("const uint16_t test_img_160x80[160x80] = {");
f.write("static const uint8_t test_img_%dx%d[%d*%d*2] = {\n" % (img.width, img.height, img.width, img.height))

cnt = 1
s = ""
for y in range(0, img.height):
    # s = ""
    for x in range(0, img.width):
        (r, g, b, _) = img.getpixel( (x, y) )
        color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3)
        # for right endiness, so ST7735_DrawImage would work
        color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8)
        s += "0x{:02X}, 0x{:02X}, ".format(color565 & 0xff, color565 >> 8)
        if cnt % 16 == 0:
            f.write(s + '\n')
            s = ""
        cnt += 1
    # print(s)

# print("};")
f.write("};")
f.close()