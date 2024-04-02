import time
from matplotlib import MatplotlibDeprecationWarning
from IMX219 import ImageSensorIMX219
from bergipy import dec2reg, array2png
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import warnings
import json
import cv2
import os

registers = {
    # Gain:
    '0157': 'FA',    # ANA_GAIN_GLOBAL_A             FD highest stable value
    '0158': '0100',  # DIG_GAIN_GLOBAL_A             0100 means Gain=1 so no effect

    # Exposure time:
    '015A': '1000',  # COARSE_INTEGRATION_TIME_A     max. working FFFC      Exposure
    '0160': 'FFFF',  # FRM_LENGTH_A                  max. working FFFF      Frameperiod
    '0162': '0D78',  # LINE_LENGTH_A                 max. working FFFF      Exposure, Frameperiod 0D78

    # Clocking
    '0304': '03',  # PREPLLCK_VT_DIV               default 03              max. working 03
    '0305': '03',  # PREPLLCK_OP_DIV               default 03              max. working 03

    '0306': '0040',  # PLL_VT_MPY                    default 002B            min. working 0008
    '030C': '0080',  # PLL_OP_MPY                    default 0055            min. working 000D

    '0301': '05',  # VTPXCK_DIV                    default 05
    '0309': '0A',  # OPPXCK_DIV                    default 0A

    # Cropping
    '0164': dec2reg(0),  # X_min
    '0166': dec2reg(3279),  # X_max     default 3279

    '0168': dec2reg(0),  # Y_min
    '016A': dec2reg(2463),  # Y_max     default 2463

    # Binning
    '0174': '00',    # BINNING_MODE_H_A              0 = None, 1 = 2x2, 2 = 4x4
}

sensor = ImageSensorIMX219()

for gain_set in ['00', 'FD'] * 10:
    registers['0157'] = gain_set

    image_stack = sensor.acquire(overwrite_registers=registers, return_frame=False, verbose=False, overwrite_on_time=0.3)
    exposure_time, camera_on_time, frame_period, gain = sensor.calculate_exposure(overwrite_registers=registers)

    print(f'Gain is {gain}')




