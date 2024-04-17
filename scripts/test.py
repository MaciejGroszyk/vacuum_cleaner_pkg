from drawImage import ImgDrawCreator, DrawImageHorizontal, DrawImageVertical
from PIL import Image, ImageDraw

if __name__ == "__main__":
    idc = ImgDrawCreator()
    img = idc.img
    draw = idc.draw
    
    dlh = DrawImageHorizontal(img, draw)
    dlv = DrawImageVertical(img, draw)
    
    dlh.drawRandomLineWithDoor()
    dlv.drawRandomLineWithDoor()

    idc.saveImage()
    