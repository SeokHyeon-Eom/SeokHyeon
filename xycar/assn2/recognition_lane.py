#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture('2.avi')

# value start point
value_threshold = 190

# ROI y-coordinate's start point
roi_vertical_pos = 300

# video "2.avi"'s x-coordinate length
image_width = 640

# To find lane from x-coordinate 0~180 to 20~200 and from 620~440 to 640~460(reverse)
scan_width = 200
lmid = scan_width  # 200
rmid = image_width - lmid  # 440

# blue square y-coordinate's length and ROI y-coordinate's length
scan_height = 20

# yellow square x-value
area_width = 20

# when find lane, y-coordinate value
area_height = 10
row_begin = (scan_height - area_height) // 2  # 5
row_end = row_begin + area_height  # 15

# How white many pixel
pixel_cnt_threshold = 0.2 * area_width * area_height

while True:
    # one frame picture
    ret, frame = cap.read()
    if not ret:
        break
    if cv2.waitKey(1) & 0xFF == 27:  # esc
        break

    # At screen, blue square
    # x-coordinate drawing range is all range
    # x-coordinate drawing range is from 300 to 320
    # color = blue, thickness = 3
    frame = cv2.rectangle(frame, (0, roi_vertical_pos),
                          (image_width - 1, roi_vertical_pos + scan_height),
                          (255, 0, 0), 3)

    # [y-s: y-e, x-s: x-e] is ROI
    # ROI(Checking) is first y-coordinate
    # y-coordinate checking range is from 300 to 320
    # x-coordinate checking range is all range
    roi = frame[roi_vertical_pos:roi_vertical_pos + scan_height, :]

    # ROI convert HSV
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
    ubound = np.array([255, 128, 255], dtype=np.uint8)

    # if pixel is in range(lbound, ubound), white otherwise black
    # convert white or black
    bin = cv2.inRange(hsv, lbound, ubound)

    # To show yellow square, convert
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1

    # for l in range(20, 200):
    for l in range(area_width, lmid):
        # y-value is fixed form 5 to 15, x-value(20) range(from 0 to 200)
        area = bin[row_begin:row_end, l - area_width:l]
        # if white color is more than pixel_cnt, left yellow square start point is l
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            left = l
            break
    # for r in range(620, 440, -1):
    for r in range(image_width - area_width, rmid, -1):
        # y-value is fixed form 5 to 15, x-value(20) range(from 620 to 440, -1)
        area = bin[row_begin:row_end, r:r + area_width]
        # if white color is more than pixel_cnt, left yellow square start point is l
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            right = r
            break

    if left != -1:
        lsquare = cv2.rectangle(view,
                                (left - area_width, row_begin),
                                (left, row_end),
                                (0, 255, 255), 2)
    else:
        print("Lost left line")

    if right != -1:
        rsquare = cv2.rectangle(view,
                                (right, row_begin),
                                (right + area_width, row_end),
                                (0, 255, 255), 2)
    else:
        print("Lost right line")

    cv2.imshow("origin", frame)
    cv2.imshow("view", view)
    # cv2.imshow("hsv",hsv)

    time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()
