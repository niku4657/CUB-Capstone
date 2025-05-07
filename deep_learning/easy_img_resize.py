from PIL import Image

#resizing random image to image size we are using
im = Image.open('crops.jpg')
im = im.resize((1920, 300)) 

#resizing but maintaining middle point
#however, this is problematic as the right and left trackers are over 300 point apart
im1 = im.crop((810, 0, 1110, 300))

im1.show()
