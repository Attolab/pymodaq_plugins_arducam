from pymodaq_plugins_arducam.hardware.IMX219Tools import dec2reg
from subprocess import Popen, PIPE
from datetime import datetime
import numpy as np
import time
import sys
import os


class ImageSensorIMX219():
    """Image sensor IMX219 control interface."""

    def __init__(self):
        self.ssh_hostname = os.getenv('sensor_ssh_hostname')

        process = Popen(['ssh', '-o', 'ConnectTimeout=3', self.ssh_hostname, 'ls'], stdout=PIPE, stderr=PIPE)
        process.communicate()

        if process.returncode != 0:
            print('ERROR - unable to connect to RPi - Is the SSH alias set?')
            exit()

        self.default_registers = {
            # Gain:
            '0157': 'FE',    # ANA_GAIN_GLOBAL_A             FD highest stable value
            '0158': '0100',  # DIG_GAIN_GLOBAL_A             0100 means Gain=1 so no effect

            # Exposure time:
            '015A': '0100',  # COARSE_INTEGRATION_TIME_A     max. working FFFC      Exposure
            '0160': 'FFFF',  # FRM_LENGTH_A                  max. working FFFF      Frameperiod
            '0162': '0D78',  # LINE_LENGTH_A                 max. working FFFF      Exposure, Frameperiod

            # Clocking
            '0304': '03',    # PREPLLCK_VT_DIV               default 03              max. working 03
            '0305': '03',    # PREPLLCK_OP_DIV               default 03              max. working 03

            '0306': '002B',  # PLL_VT_MPY                    default 002B            min. working 0008
            '030C': '0055',  # PLL_OP_MPY                    default 0055            min. working 000D

            '0301': '05',    # VTPXCK_DIV                    default 05
            '0309': '0A',    # OPPXCK_DIV                    default 0A

            # Binning
            '0174': '00',    # BINNING_MODE_H_A              0 = None, 1 = 2x2, 2 = 4x4
            '0175': '00',    # BINNING_MODE_V_A

            # Cropping
            '0164': dec2reg(0),       # X_min
            '0166': dec2reg(3279),    # X_max     default 3279

            '0168': dec2reg(0),       # Y_min
            '016A': dec2reg(2463),    # Y_max     default 2463
        }

        self.supported_cropping_modes = [(3280, 2464), (1920, 1080), (256, 256), (128, 128)]

        self.abs_directory_path = os.path.abspath(os.path.dirname(__file__))

    def acquire(self, overwrite_registers=None, overwrite_on_time=None, persist_on_local=False,
                return_frame=False, verbose=False):
        """Acquire a single frame from the sensor.

        Function that instructs the remote machine in the sample holder
        to acquire a frame from the CMOS sensor. The captured frame is
        automatically timestamped and saved on the remote machine as a
        .bayerraw file. Optionally, it can also be returned by this
        function - however this will add 2-3s of transfer time depending
        on network bandwidth.

        Parameters
        ----------
        overwrite_registers : dict
            Dictionary containing sensor registers that are different from the
            standard configuration.
        overwrite_on_time : float
            If given, operates the sensor for the specified amount of time.
        return_frame : bool
            Default is False. Set to True to return numpy array of pixel data.
            This will add 2-3s of transfer and processing time
        verbose : bool
            Default is False. Set to True for increased verbosity.

        Returns
        -------
        filepath : str
            By default the filepath of the acquired frame on the remote machine is returned.
        frame : ndarray
            If return_frame is set, a 3-dimensional numpy array containing the pixel values
            for each acquired frame will be returned. The data type is uint16 and the shape
            depends on the pixel binning and cropping settings of the sensor. No pixel binning
            results in shape (2464, 3280), 2x2 in shape (1232, 1640) and 4x4 in shape (616, 820).
            In this case the packed frame data will be automatically stored on the local machine.
        """

        registers = self.default_registers

        if overwrite_registers:
            for key, value in overwrite_registers.items():
                registers[key] = value

        if registers['0174'] != registers['0175']:
            print('ERROR - Vertical and Horizontal binning are different. This is not supported.')
            exit()

        binning_mode = int(registers['0174'])

        if binning_mode == 0:
            frame_resolution = 2464, 3280
        elif binning_mode == 1:
            frame_resolution = 1232, 1640
        elif binning_mode == 2:
            frame_resolution = 616, 820
        else:
            print('ERROR - Unsupported binning mode.')
            exit()

        frame_size_x = int(registers['0166'], base=16) - int(registers['0164'], base=16) + 1
        frame_size_y = int(registers['016A'], base=16) - int(registers['0168'], base=16) + 1

        if (frame_size_x, frame_size_y) != (3280, 2464) and binning_mode != 0:
            print('ERROR - Simultaneous binning and cropping is not supported.')
            exit()

        if (frame_size_x, frame_size_y) not in self.supported_cropping_modes:
            print(f'ERROR - Cropping to size {(frame_size_x, frame_size_y)} is not supported.')
            exit()

        registers['016C'] = dec2reg(frame_size_x)
        registers['016E'] = dec2reg(frame_size_y)

        if binning_mode == 0:
            frame_resolution = (frame_size_y, frame_size_x)     # TODO unify frame_resolution and frame_size

            if (frame_size_x, frame_size_y) != (3280, 2464):
                binning_mode = frame_size_x

        if 'darwin' or 'linux' in sys.platform:
            registers_str = '\;'.join([f'{k},{v}' for k, v in registers.items()])
        else:
            registers_str = ';'.join([f'{k},{v}' for k, v in registers.items()])

        exposure_time, camera_on_time, frame_period, gain = self.calculate_exposure(registers)

        if overwrite_on_time is not None:
            camera_on_time = overwrite_on_time

        if verbose:
            print(f'Expected exposure time is {exposure_time:.6f}s')
            print(f'Expected camera-on time is {camera_on_time * 1000:.0f}ms')
            print(f'Expected frame period is {frame_period * 1000:.0f}ms')
            print(f'Expected frame rate is {1 / frame_period:.2f}fps\n')

        number_of_frames = int(camera_on_time / frame_period)
        if number_of_frames > 60:
            print('ERROR - Too many frames for RAM')
            exit()

        timestamp = datetime.now().strftime('%d.%m._%H-%M-%S')

        cmd = ['ssh',
               self.ssh_hostname,
               '/home/pi/raspiraw_custom/raspiraw',
               '-md 0',
               '-y 10',
               '-sr 1',
               f'--width {frame_size_x}',
               f'--height {frame_size_y}',
               f'--regs "{registers_str}"',
               f'-t {camera_on_time * 1000}',
               f'-o /dev/shm/frame_%04d.bayerraw']

        if verbose:
            print(' '.join(cmd))
            start = time.perf_counter()
            pipe_stderr = None
        else:
            pipe_stderr = PIPE

        process = Popen(' '.join(cmd), stdout=PIPE, stderr=pipe_stderr, shell=True)
        process.communicate()

        if verbose or return_frame or persist_on_local:
            list_cmd = ['ssh',
                        self.ssh_hostname,
                        'find',
                        f'/dev/shm/frame_*.bayerraw',
                        '-type f',
                        '-size +1',
                        '-mmin 0.3']

            process = Popen(list_cmd, stdout=PIPE, stderr=pipe_stderr)
            stdout, stderr = process.communicate()
            frames_filenames = stdout.decode().split('\n')[:-1]

            if verbose:
                print(f'{len(frames_filenames)} Frame(s) acquired. Took {time.perf_counter() - start}s')
                start = time.perf_counter()

        if persist_on_local:
            transfer_cmd = ['ssh', self.ssh_hostname, '"']

            for idx, frame_filename in enumerate(frames_filenames):
                transfer_cmd += ['dd',
                                 f'bs=10171392',
                                 f'if={frame_filename}',
                                 f'of=/home/pi/frames/{timestamp}_{idx:>04}.bayerraw',
                                 '&&']

            transfer_cmd = transfer_cmd[:-1] + ['"']

            if verbose:
                print(f'Making {len(frames_filenames)} Frames persistent...')

            process = Popen(' '.join(transfer_cmd), stdout=PIPE, stderr=pipe_stderr, shell=True)
            process.communicate()

        if not return_frame:
            if persist_on_local:
                return f'/home/pi/frames/{timestamp}_*.bayerraw'
            else:
                return '/dev/shm/frame_*.bayerraw'

        transfer_cmd = ['ssh', self.ssh_hostname, '"']

        for frame_filename in frames_filenames:
            transfer_cmd += ['dd',
                             f'skip={0}',
                             f'count={frame_resolution[0]}',
                             f'bs={4128}',
                             f'if={frame_filename}',
                             '&&']

        transfer_cmd = transfer_cmd[:-1] + ['"']

        if verbose:
            print(f'Transferring {len(frames_filenames)} Frames...')

        process = Popen(' '.join(transfer_cmd), stdout=PIPE, stderr=pipe_stderr, shell=True)
        dd_stdout, dd_stderr = process.communicate()

        image_stack = np.zeros((len(frames_filenames), frame_resolution[0], frame_resolution[1]), dtype=np.uint16)
        bytes_per_frame = int(len(dd_stdout) / len(frames_filenames))

        for frame_idx, _ in enumerate(frames_filenames):
            image_array = self.unpack(dd_stdout[:bytes_per_frame])
            dd_stdout = dd_stdout[bytes_per_frame:]

            image_stack[frame_idx,:,:] = image_array

        if verbose:
            print(f'Transferred and unpacked {len(frames_filenames)} Frames in {time.perf_counter() - start}s')

        del image_array, stdout, stderr
        return image_stack

    def unpack(self, buffer):
        data_array = np.frombuffer(buffer, dtype=np.uint8)
        data_array = data_array.reshape((2464, 4128))

        data_array = data_array[:, :-28]
        data_array = data_array.reshape((2464, 820, 5))

        data_array = data_array.astype(np.uint16)
        data_array[:, :, :-1] = data_array[:, :, :-1] << 2

        data_array[:, :, 3] |= data_array[:, :, 4] & 3
        data_array[:, :, 2] |= (data_array[:, :, 4] >> 2) & 3
        data_array[:, :, 1] |= (data_array[:, :, 4] >> 4) & 3
        data_array[:, :, 0] |= (data_array[:, :, 4] >> 6)

        data_array = data_array[:, :, :4]
        data_array = data_array.reshape((2464, 3280))

        return data_array

    def get_sensor_temperature(self):
        """Get sensor temperature.

        Read the temperature of the sensor as measured by its internal
        temperature probe.

        Returns
        -------
        temperature : int
            Measured temperature in °C. Sensor accuracy is +/- 5°C.

        """

        cmd = ['ssh',
               self.ssh_hostname,
               'python',
               '/home/pi/XUV-acquisition-repo/src/XUV_acquisition/i2c_readout_local.py']

        process = Popen(cmd, stdout=PIPE)
        stdout, stderr = process.communicate()
        value = stdout.decode()

        temperature = np.interp(value, (0, 128), (-10, 95))
        temperature = round(temperature)

        return temperature

    def get_cpu_temperature(self):
        """Get CPU temperature.

        Read the temperature of the remote machine's CPU that is inside of the
        vacuum system.

        Returns
        -------
        temperature : float
            Measured temperature in °C.

        """

        cmd = ['ssh',
               self.ssh_hostname,
               'vcgencmd',
               'measure_temp']

        process = Popen(cmd, stdout=PIPE)
        stdout, stderr = process.communicate()

        temperature = float(stdout[5:9])
        return temperature

    def calculate_exposure(self, overwrite_registers=None):
        """TODO: Update Documentation"""
        """Calculate frame exposure time.

        Calculate Frame exposure time and camera-on time based on sensor
        register values.

        Parameters
        ----------
        overwrite_registers : dict
            Dictionary containing the sensor register address and it's value in
            hexadecimal. It needs to contain the values of the following
            registers: 015A, 0162, 0304, 0306, 0301, 0305, 030C, 0309.

        Returns
        -------
        exposure time : float
            The exposure time of the frame in ms.
        camera_on_time : float
            The minimum amount of time in ms the camera has to be operating to
            facilitate the capture of one frame at that exposure time.

        """

        registers = self.default_registers

        if overwrite_registers is not None:
            for key, value in overwrite_registers.items():
                registers[key] = value

        coarse_integration_time = int(registers['015A'], base=16)
        frame_length_lines = int(registers['0160'], base=16)
        line_length = int(registers['0162'], base=16)
        fine_integration_time = 500

        pre_div_1 = int(registers['0304'], base=16)
        mult_pll_1 = int(registers['0306'], base=16)
        pix_div_1 = int(registers['0301'], base=16)

        pre_div_2 = int(registers['0305'], base=16)
        mult_pll_2 = int(registers['030C'], base=16)
        pix_div_2 = int(registers['0309'], base=16)

        sensor_input_clock_freq = 24 * 10 ** 6
        pixel_clock_freq = sensor_input_clock_freq * 1 / pre_div_1 * mult_pll_1 * 1 / pix_div_1
        output_clock_freq = sensor_input_clock_freq * 1 / pre_div_2 * mult_pll_2 * 1 / pix_div_2

        if frame_length_lines - 4 >= coarse_integration_time: frame_length = frame_length_lines
        elif frame_length_lines - 4 < coarse_integration_time: frame_length = coarse_integration_time + 4

        time_per_line = line_length / (2 * pixel_clock_freq)
        frame_period = time_per_line * frame_length

        exposure_time = (coarse_integration_time * line_length / 2 + fine_integration_time / 2) * 1 / pixel_clock_freq
        clockout_time = 1 / output_clock_freq * 7 * 10 ** 7
        camera_on_time = 0.5 + exposure_time + clockout_time

        gain = 256 / (256 - int(self.default_registers['0157'], base=16))

        return exposure_time, camera_on_time, frame_period, gain

    def set_sensor_parameters(self, gain=None, exposure_time=None):
        """Set Sensor Parameters Easy Mode

        Configure the sensor registers for a desired gain and exposure time.
        Note that this function will set the registers in the most naive way
        and does not offer the full functionality of the sensor.

        Parameters
        ----------
        gain : float
            Setting the analoge gain of the sensor, corresponding to a linear increase
            in sensitivity. The maximum stable value is 85.

        exposure_time: float
            Desired Exposure time in seconds.

        Returns
        -------
        Nothing
        """

        if gain is not None:
            gain_register_value = 256 - (256 / gain)
            self.default_registers['0157'] = f"{int(gain_register_value):#0{4}x}"[2:]

        if exposure_time is not None:
            line_length = int(self.default_registers['0162'], base=16)
            fine_integration_time = 500

            pre_div_1 = int(self.default_registers['0304'], base=16)
            mult_pll_1 = int(self.default_registers['0306'], base=16)
            pix_div_1 = int(self.default_registers['0301'], base=16)

            sensor_input_clock_freq = 24 * 10 ** 6
            pixel_clock_freq = sensor_input_clock_freq * 1 / pre_div_1 * mult_pll_1 * 1 / pix_div_1

            coarse_integration_time = (exposure_time * pixel_clock_freq - fine_integration_time / 2) / (line_length / 2)

            if coarse_integration_time > 0xFFFC:
                print('Desired exposure time is too large. Try to reduce clock speed or increase line length. Exiting...')
                exit()

            if coarse_integration_time < 1:
                print('Desired exposure time is too small. Try to increase clock speed or reduce line length. Exiting...')
                exit()

            self.default_registers['015A'] = f"{int(coarse_integration_time):#0{4}x}"[2:]