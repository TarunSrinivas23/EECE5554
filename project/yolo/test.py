import cv2
import numpy as np
import os
from ultralytics import YOLO

model = YOLO("yolov8l-seg.pt")


img = cv2.imread("/home/tarun/northeastern/RSN/project/bagtoimg/project_data/image_0/000684.png")
print(img.shape) #1224(x), 1024(y)
img = cv2.resize(img, (640, 544))

results = model(img)
# print(results)
height = 544
width = 640

# Create a blank black image
blank_image = np.zeros((height, width), dtype=np.uint8)

for result in results:
    boxes = result.boxes  # Boxes object for bbox outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs

    im_array = result.plot(boxes = False, masks = True, probs = False)
    cv2.imshow("test", im_array)
    cv2.waitKey(0)
    for i in range(len(masks)):
        mask = masks[i].xy[0]
        mask = np.array([mask], dtype=np.int32)
        mask = mask.reshape((-1, 1, 2))
        cv2.fillPoly(blank_image, [mask], 255)
    cv2.imshow("test", blank_image)
    cv2.waitKey(0)
