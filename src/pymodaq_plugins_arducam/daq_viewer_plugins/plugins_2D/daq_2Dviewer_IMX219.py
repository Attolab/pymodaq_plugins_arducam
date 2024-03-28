from pymodaq.utils.daq_utils import ThreadCommand
from pymodaq.utils.data import DataFromPlugins, Axis, DataToExport
from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters, main
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_arducam.hardware.IMX219 import ImageSensorIMX219
from pymodaq_plugins_arducam.hardware.IMX219Tools import dec2reg, array2png
import numpy as np
from qtpy import QtWidgets, QtCore


class DAQ_2DViewer_IMX219(DAQ_Viewer_base):
    """ Instrument plugin class for a 2D viewer.
    
    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Viewer module through inheritance via
    DAQ_Viewer_base. It makes a bridge between the DAQ_Viewer module and the Python wrapper of a particular instrument.

    TOD Complete the docstring of your plugin with:
        * The set of instruments that should be compatible with this instrument plugin.
        * With which instrument it has actually been tested.
        * The version of PyMoDAQ during the test.
        * The version of the operating system.
        * Installation instructions: what manufacturer’s drivers should be installed to make it run?

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library.
         
    # TOD add your particular attributes here if any

    """
    params = comon_parameters + [
        {'title': 'Camera:', 'name': 'camera_list', 'type': 'list', 'limits': []},
        {'title': 'Camera model:', 'name': 'camera_info', 'type': 'str', 'value': '', 'readonly': True},
        {'title': 'Update ROI', 'name': 'update_roi', 'type': 'bool_push', 'value': False},
        {'title': 'Clear ROI+Bin', 'name': 'clear_roi', 'type': 'bool_push', 'value': False},
        {'title': 'Binning', 'name': 'binning', 'type': 'list', 'limits': [1, 2, 4]},
        {'title': 'Image width', 'name': 'hdet', 'type': 'int', 'value': 1, 'readonly': True},
        {'title': 'Image height', 'name': 'vdet', 'type': 'int', 'value': 1, 'readonly': True},
        {'title': 'Cropping mode', 'name': 'crop_mode', 'type': 'list', 'limits': ['(3280, 2464)', '(1920, 1080)', '(256, 256)', '(128, 128)']},
        {'title': 'Gain', 'name': 'gain', 'type': 'int', 'value': 1, 'readonly': False},
        {'title': 'Timing', 'name': 'timing_opts', 'type': 'group', 'children':
            [{'title': 'Exposure Time (s)', 'name': 'exposure_time', 'type': 'float', 'value': 1.0},
             {'title': 'Camera On Time (s)', 'name': 'camera_on_time', 'type': 'float', 'value': 1.0, 'readonly': True},
             {'title': 'Frame Period', 'name': 'frame_period', 'type': 'float', 'value': 0.0, 'readonly': True}]
         }
    ]
    start_acquire_signal = QtCore.Signal(int)   #Signal with number of required frames
    stop_acquire_signal = QtCore.Signal()
    roi_pos_size = QtCore.QRectF(0,0,10,10)

    def ini_attributes(self):
        self.controller: ImageSensorIMX219 = None
        self.x_axis = None
        self.y_axis = None

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        # TODO for your custom plugin
        if param.name() == "a_parameter_you've_added_in_self.params":
            self.controller.your_method_to_apply_this_param_change()
        #elif ...

    def ini_detector(self, controller=None):
        """Detector communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator/detector by controller
            (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        self.ini_detector_init(old_controller=controller,
                               new_controller=ImageSensorIMX219())
        self.controller.set_sensor_parameters(gain=self.settings['gain'],
                                              exposure_time=self.settings['timing_opts', 'exposure_time'])
        exposure_time, camera_on_time, frame_period, gain = self.controller.calculate_exposure()
        self.settings.child("gain").setValue(gain)
        self.settings.child("timing_opts", "exposure_time").setValue(exposure_time)
        self.settings.child("timing_opts", "camera_on_time").setValue(camera_on_time)
        self.settings.child("frame_period").setValue(frame_period)


        info = "Initialized camera"
        initialized = True
        return info, initialized

    def close(self):
        """Terminate the communication protocol"""
        ## TODO for your custom plugin
        raise NotImplemented  # when writing your own plugin remove this line
        #  self.controller.your_method_to_terminate_the_communication()  # when writing your own plugin replace this line

    def grab_data(self, Naverage=1, **kwargs):
        """Start a grab from the detector

        Parameters
        ----------
        Naverage: int
            Number of hardware averaging (if hardware averaging is possible, self.hardware_averaging should be set to
            True in class preamble and you should code this implementation)
        kwargs: dict
            others optionals arguments
        """
        self.Naverage = Naverage
        if 'live' in kwargs:
            self.live = kwargs['live']
        if 'wait_time' in kwargs:
            self.wait_time = kwargs['wait_time']

        if self.live:
            self.start_acquire_signal.emit(-1)  # will trigger the waitfor acquisition
        else:
            self.start_acquire_signal.emit(Naverage)  # will trigger the waitfor acquisition


        # image_stack = self.controller.acquire(overwrite_registers=self.registers, return_frame=True, verbose=False)
        # data = np.mean(image_stack, axis=0)
        # frame_size_x = int(self.registers['0166'], base=16) - int(self.registers['0164'], base=16) + 1
        # frame_size_y = int(self.registers['016A'], base=16) - int(self.registers['0168'], base=16) + 1
        #
        # self.x_axis = Axis(data=np.linspace(0, frame_size_x, frame_size_x, endpoint=False), label='Pixels', index = 1)
        # self.y_axis = Axis(data=np.linspace(0, frame_size_y, frame_size_y, endpoint=False), label='Pixels', index=0)
        #
        # self.dte_signal.emit(DataToExport('IMX219',
        #                                   data=[DataFromPlugins(name='IMX219', data=[data],
        #                                                         dim='Data2D', labels=['Camera image'],
        #                                                         x_axis=self.x_axis,
        #                                                         y_axis=self.y_axis), ]))
        #

    def stop(self):
        """Stop the current grab hardware wise if necessary"""
        return ''


if __name__ == '__main__':
    main(__file__)
