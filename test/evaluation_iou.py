
import io
import numpy as np    
from PIL import Image


# Adapted from: https://stackoverflow.com/questions/66595055/fastest-way-of-computing-binary-mask-iou-with-numpy
def binaryMaskIOU(mask1, mask2):
    intersection = np.count_nonzero(np.logical_and(mask1, mask2))
    iou = intersection / (np.count_nonzero(mask1) + np.count_nonzero(mask2) - intersection)
    return iou

with open('../resources/evaluation/gt_mask.jpg', 'rb') as gt, open('../resources/evaluation/grabcut_mask.jpg', 'rb') as grabcut, open('../resources/evaluation/color_threshold_mask.jpg', 'rb') as color_threshold, open('../resources/evaluation/u2net_mask.jpg', 'rb') as u2net:
    gt_mask = np.asarray(Image.open(io.BytesIO(gt.read())))
    grabcut_mask = np.asarray(Image.open(io.BytesIO(grabcut.read())))
    color_threshold_mask = np.asarray(Image.open(io.BytesIO(color_threshold.read())))
    u2net_mask = np.asarray(Image.open(io.BytesIO(u2net.read())))

    print("IOU_grabcut = ", binaryMaskIOU(grabcut_mask, gt_mask))
    print("IOU_color_threshold = ", binaryMaskIOU(color_threshold_mask, gt_mask))
    print("IOU_u2net = ", binaryMaskIOU(u2net_mask, gt_mask))
