from eyepy import *

resolution = QQVGA
CAMInit(resolution)

while True:
    image = CAMGetGray()
    LCDImageGray(image)
    LCDImageGray(Image.from_list([255 - x for x in image], gray=True, resolution=resolution), start=(0, resolution.HEIGHT + 1))
    LCDImageGray(Image.from_list([x + 128 for x in image], gray=True, resolution=resolution), start=(resolution.WIDTH + 1, resolution.HEIGHT + 1))
