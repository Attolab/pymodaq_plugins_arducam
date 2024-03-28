import numpy as np
import cv2


def dec2reg(dec_value):
    return str(hex(dec_value)[2:].zfill(4))


def array2png(array, filename):
    array = array.astype(np.uint16)
    cv2.imwrite(filename, array << 6)


def png2array(filename):
    return cv2.imread(filename, cv2.IMREAD_UNCHANGED) >> 6