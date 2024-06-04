import cv2 as cv
import numpy as np
from PIL import Image
from PIL import ImageEnhance

im=Image.open(r"./data/1/test1.jpg")
en=ImageEnhance.Brightness(im)
im1=en.enhance(2)
im1.save("./data/1/out1.jpg")
im=Image.open(r"./data/1/out1.jpg")
en=ImageEnhance.Sharpness(im)
im1=en.enhance(2)
im1.save("./data/1/out2.jpg")

src = cv.imread("./data/1/out3.jpg")
cv.imshow("input", src)
src = cv.GaussianBlur(src, (3, 3), 0)
gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
edge = cv.Canny(src, 65, 88)

se = cv.getStructuringElement(cv.MORPH_ELLIPSE, (10, 10))
binary = cv.morphologyEx(edge, cv.MORPH_CLOSE, se) # 先膨胀再腐蚀
contours, hireachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
count = 0

areas = []
for c in range(len(contours)):
    areas.append(cv.contourArea(contours[c]))
max_id = areas.index(max(areas))
max_rect = cv.minAreaRect(contours[max_id])
max_box = cv.boxPoints(max_rect)
max_box = np.int0(max_box)
res = cv.drawContours(src, [max_box], 0, (255, 0, 0), 2)

cv.imshow("binary", binary)
cv.imshow("result", src)
cv.waitKey(0)
cv.destroyAllWindows()