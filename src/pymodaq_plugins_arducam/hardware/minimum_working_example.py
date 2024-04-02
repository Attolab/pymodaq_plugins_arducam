from IMX219 import ImageSensorIMX219
import matplotlib.pyplot as plt


sensor = ImageSensorIMX219()

# Binning Example
sensor.set_sensor_parameters(gain=86, exposure_time=1, binning=1)

# Windowing Example
sensor.set_sensor_parameters(gain=86, exposure_time=1, window={'wsize': 256, 'xmin': 500, 'ymin': 600})

image_stack = sensor.acquire(return_frame=True, verbose=True)

exposure_time, camera_on_time, frame_period, gain = sensor.calculate_exposure()
temperature = sensor.get_sensor_temperature()

print(f'Sensor temperature is {temperature}')
print(f'Actual exposure time is {exposure_time}')
print(f'Actual frame period is {frame_period}')
print(f'Actual gain is {gain}')

for image in image_stack:
    plt.imshow(image, cmap='gray', vmin=0, vmax=1023, interpolation=None)
    plt.show()
