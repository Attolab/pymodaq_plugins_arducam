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

warnings.filterwarnings("ignore", category=MatplotlibDeprecationWarning)

registers = {
    # Gain:
    '0157': 'FD',    # ANA_GAIN_GLOBAL_A             FD highest stable value
    '0158': '0100',  # DIG_GAIN_GLOBAL_A             0100 means Gain=1 so no effect

    # Exposure time:
    '015A': 'F000',  # COARSE_INTEGRATION_TIME_A     max. working FFFC      Exposure
    '0160': '0100',  # FRM_LENGTH_A                  max. working FFFF      Frameperiod
    '0162': '2000',  # LINE_LENGTH_A                 max. working FFFF      Exposure, Frameperiod 0D78

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
    '0174': '02',    # BINNING_MODE_H_A              0 = None, 1 = 2x2, 2 = 4x4
}

sensor = ImageSensorIMX219()

filename = 'XUV-burn-in-test'

timestamp = datetime.now().strftime('%d.%m._%H:%M:%S')
filepath = f'../../manual_acquisitions/{timestamp}-{filename}'
os.mkdir(filepath)

idx = 0

while True:
    image_stack = sensor.acquire(overwrite_registers=registers, return_frame=True, verbose=True)#, overwrite_on_time=1.4)
    exposure_time, camera_on_time, frame_period, gain = sensor.calculate_exposure(overwrite_registers=registers)
    temperature = sensor.get_sensor_temperature()

    print(f'Temp is {temperature}')
    print(f'Gain is {gain}')

    with open(f'{filepath}/meta_{idx}.txt', 'w') as f:
        f.write(json.dumps({'Exposure Time': exposure_time, 'Gain': int(registers['0157'], base=16)}))
        f.write(f'\nUnix TS: {time.time()}')
        f.write(f'\nSensor Temp: {temperature}')
        f.write('\n')
        f.write(json.dumps(registers))

    for image_idx, image in enumerate(image_stack):
        array2png(image, f'{filepath}/image_{idx}_{image_idx}.png')

        # plt.hist(image.flatten(), bins=min(int(np.max(image)), 2000))
        # plt.semilogy()
        # plt.savefig(f'{filepath}/hist_{idx}_{image_idx}')
        # plt.clf()

    plt.style.use('dark_background')
    plt.tight_layout()
    plt.imshow(image_stack[0,:,:], cmap='gray', vmin=0, vmax=1023, interpolation=None)
    plt.show()

    idx += 1


