from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main  # common set of parameters for all actuators
from pymodaq.daq_utils.daq_utils import ThreadCommand, getLineInfo  # object used to send info back to the main thread
from pymodaq.daq_utils.parameter import Parameter
from pymodaq.daq_utils.config import Config
import time
config = Config()

try:
    import smaract.ctl as ctl
except ImportError:
    raise RuntimeError("The Smaract Python SDK was not found.")

"""
    =================================================================================
    This plugin handles SmarAct MCS2 controller, using the Python SDK provided by Smaract.
    The python SDK is only present in newer installers and has special install instructions, see 
    the documentation file "Using SmarAct Python SDKs.pdf".

    If you use the first version of MCS controller use the daq_move_SmarActMCS
    plugin instead.
    The SmarAct MCS2 installer should be executed for this plugin to work.
    We suppose that the configuration of the controller and the positioners
    (sensor type…) has been done via the SmarAct MCS2ServiceTool software.

    If the controller is not switched on, the plugin will not be suggested in
    the list in the GUI of the daq_move.

    ==================================================================================
"""


def ctlerr(func_name, *args):
    # By default, the SDK functions do not return errors, so we need to catch them.
    # This is a little hack to call the library function with a try/error clause everytime, instead of typing it
    # Instead of calling "ctl.Move(d_handle, channel, value, 0)"
    # We can use "ctlerr('Move', d_handle, channel, value, 0)". This will execute the command, and raise
    # a runtime error if the SDK function finds an error.
    def func_not_found():  # just in case we dont have the function
        raise RuntimeError('Function '+func_name+' was not found in the library.')

    func = getattr(ctl, func_name, func_not_found)
    try:
        return func(*args)
    except ctl.Error as e:
        raise RuntimeError(e)


def assert_lib_compatibility():
    """
    Checks that the major version numbers of the Python API and the
    loaded shared library are the same to avoid errors due to
    incompatibilities.
    Raises a RuntimeError if the major version numbers are different.
    """
    vapi = ctl.api_version
    vlib = [int(i) for i in ctl.GetFullVersionString().split('.')]
    if vapi[0] != vlib[0]:
        raise RuntimeError("Incompatible SmarActCTL python api and library version.")


version = ctl.GetFullVersionString()
assert_lib_compatibility()

buffer = ctlerr('FindDevices')
if len(buffer) == 0:
    raise RuntimeError("No Smaract MCS2 devices found.")
controller_locators = buffer.split("\n")


class DAQ_Move_SmarActMCS2_PySDK(DAQ_Move_base):
    """
        =============== ==============
        **Attributes**    **Type**
        *params*          dictionnary
        =============== ==============
    """
    _controller_units = 'µm'
    is_multiaxes = True  # we suppose a have a MCS2 controller with a sensor
    # module for 3 channels (stages).

    axes_names = [0, 1, 2]  # be careful that the channel index starts at 0
    # and not at 1 has is done in MCS2ServiceTool

    # bounds corresponding to the SLC-24180. Will be used at default if user doesn't provide other ones.
    min_bound = -61500  # µm
    max_bound = +61500  # µm

    offset = 0  # µm
    move_mode = None
    step_position = 0

    params = [
                 {'title': 'Controller parameters:',
                  'name': 'controller_parameters',
                  'type': 'group',
                  'children': [
                      {'title': 'Smaract Library:',
                       'name': 'smaract_library',
                       'type': 'str',
                       'value': version,
                       'readonly': True},
                      {'title': 'Controller Name:',
                       'name': 'smaract_mcs2',
                       'type': 'str',
                       'value': 'SmarAct MCS2 controller',
                       'readonly': True},
                      {'title': 'Controller locator',
                       'name': 'controller_locator', 'type': 'list',
                       'limits': controller_locators},
                  ]},
                 {'title': 'Stage parameters:',
                  'name': 'stage_parameters',
                  'type': 'group',
                  'children': [
                      {'title': 'Is referenced:',
                       'name': 'is_referenced',
                       'type': 'led',
                       'value': False},
                      {'title': 'Is calibrated:',
                       'name': 'is_calibrated',
                       'type': 'led',
                       'value': False},
                      {'title': 'Max. Closed Loop Frequency (Hz):',
                       'name': 'max_cl_freq',
                       'type': 'int',
                       'value': 18500},
                      {'title': 'Holding time (ms):',
                       'name': 'hold_time',
                       'type': 'int',
                       'value': 1000},
                      {'title': 'Quiet mode:',
                       'name': 'quiet_mode',
                       'type': 'bool',
                       'value': False},
                      {'title': 'Move velocity (mm/s):',
                       'name': 'move_velocity',
                       'type': 'int',
                       'value': 10},
                      {'title': 'Move acceleration (mm/s2):',
                       'name': 'move_acceleration',
                       'type': 'int',
                       'value': 100},
                      {'title': 'Step frequency (Hz):',
                       'name': 'step_frequency',
                       'type': 'int',
                       'value': 1000,
                       'min': 1,
                       'max': 20000},
                      {'title': 'Step amplitude',
                       'name': 'step_amplitude',
                       'type': 'int',
                       'value': 65535,
                       'min': 0,
                       'max': 65535}
                  ]}
             ] + comon_parameters_fun(is_multiaxes, axes_names)

    def ini_attributes(self):
        self.controller: ctl.Open = None

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        # min and max bounds will depend on which positionner is plugged.
        # In case the user hasn't specified different values in the preset,
        # we add default values for convenience
        if self.settings.child('epsilon').value() == config('actuator', 'epsilon_default'):
            self.settings.child('epsilon').setValue(0.005)  # this means that we
            # tolerate an error of 5 nanometers on the target position

        self.settings.child('bounds', 'is_bounds').setValue(True)
        if self.settings.child('bounds', 'min_bound').value() == 0:
            self.settings.child('bounds', 'min_bound').setValue(self.min_bound)

        if self.settings.child('bounds', 'max_bound').value() == 1:
            self.settings.child('bounds', 'max_bound').setValue(self.max_bound)

        try:
            self.ini_stage_init(old_controller=controller,
                                new_controller=ctlerr('Open', self.settings.child('controller_parameters',
                                                                            'controller_locator').value()))

            channel = self.settings.child('multiaxes', 'axis').value()
            channel_state = ctlerr('GetProperty_i32', self.controller, channel, ctl.Property.CHANNEL_STATE)

            # Check if the stage has a sensor
            self.has_sensor = ctl.ChannelState.SENSOR_PRESENT & channel_state

            if self.has_sensor:
                channel_type = ctlerr('GetProperty_i32', self.controller, channel, ctl.Property.CHANNEL_TYPE)
                if channel_type == ctl.ChannelModuleType.STICK_SLIP_PIEZO_DRIVER:
                    # Set max closed loop frequency (maxCLF) to 6 kHz. This properties sets a limit for the maximum actuator driving frequency.
                    # The maxCLF is not persistent thus set to a default value at startup.
                    ctlerr('SetProperty_i32',self.controller, channel, ctl.Property.MAX_CL_FREQUENCY,
                                        self.settings.child("stage_parameters", 'max_cl_freq').value())
                    # The hold time specifies how long the position is actively held after reaching the target.
                    # This property is also not persistent and set to zero by default.
                    # A value of 0 deactivates the hold time feature, the constant ctl.HOLD_TIME_INFINITE sets the time to infinite.
                    # (Until manually stopped by "Stop") Here we set the hold time to 1000 ms.
                    ctlerr('SetProperty_i32',self.controller, channel, ctl.Property.HOLD_TIME,
                                        self.settings.child("stage_parameters", 'hold_time').value())
                elif channel_type == ctl.ChannelModuleType.MAGNETIC_DRIVER:
                    # Enable the amplifier (and start the phasing sequence).
                    ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.AMPLIFIER_ENABLED, ctl.TRUE)
                    self.settings.child("stage_parameters", 'hold_time').hide()
                    self.settings.child("stage_parameters", 'max_cl_freq').hide()
                else:
                    self.settings.child("stage_parameters", 'hold_time').hide()
                    self.settings.child("stage_parameters", 'max_cl_freq').hide()

                self.move_mode = ctl.MoveMode.CL_ABSOLUTE
                ctlerr('SetProperty_i64', self.controller, channel, ctl.Property.MOVE_VELOCITY,
                       round(self.settings.child("stage_parameters", 'move_velocity').value()*1e9))
                ctlerr('SetProperty_i64', self.controller, channel, ctl.Property.MOVE_ACCELERATION,
                       round(self.settings.child("stage_parameters", 'move_acceleration').value()*1e9))
                self.settings.child("stage_parameters", 'step_frequency').hide()
                self.settings.child("stage_parameters", 'step_amplitude').hide()

                # Check referencing
                self.settings.child('stage_parameters', 'is_referenced').setValue(
                    ctl.ChannelState.IS_REFERENCED & channel_state)
                self.settings.child('stage_parameters', 'is_calibrated').setValue(
                    ctl.ChannelState.IS_CALIBRATED & channel_state)

        # No sensor
            else:
                self.move_mode = ctl.MoveMode.STEP
                ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.STEP_FREQUENCY,
                       self.settings.child("stage_parameters", 'step_frequency').value())
                ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.STEP_AMPLITUDE,
                       self.settings.child("stage_parameters", 'step_amplitude').value())
                self.settings.child("stage_parameters", 'max_cl_freq').hide()
                self.settings.child("stage_parameters", 'hold_time').hide()
                self.settings.child("stage_parameters", 'move_acceleration').hide()
                self.settings.child("stage_parameters", 'move_velocity').hide()
                self.settings.child('stage_parameters', 'is_referenced').hide()
                self.settings.child('stage_parameters', 'is_calibrated').hide()

            # Set Move Mode
            ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.MOVE_MODE, self.move_mode)

            # Get units
            if self.has_sensor:
                if ctlerr('GetProperty_i32', self.controller, channel, ctl.Property.POS_BASE_UNIT) == ctl.BaseUnit.METER:
                    self.controller_units = 'µm'    # linear stages output picometers but we will convert to microns
                elif ctlerr('GetProperty_i32', self.controller, channel, ctl.Property.POS_BASE_UNIT) == ctl.BaseUnit.DEGREE:
                    self.controller_units = 'mdeg'  # rotationary stages output nanodegrees but we will convert it to mdeg
            else:
                self.controller_units = 'steps'
                self.step_position = 0

            initialized = True
            info = "Smaract stage initialized"

        except Exception as e:
            info = "Error: " + str(e)
            initialized = False

        return info, initialized


    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        channel = self.settings.child('multiaxes', 'axis').value()

        if self.has_sensor:
            pos = ctlerr('GetProperty_i64', self.controller, channel, ctl.Property.POSITION)
            pos = float(pos) / 1e6  # the position given by the
            # controller is in picometers, we convert in micrometers
        else:
            pos = self.step_position

        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Close the communication with the MCS2 controller.
        """
        if self.controller is not None:
            ctlerr('Close', self.controller)
        self.controller = None

    def commit_settings(self, param: Parameter):
        """
            | Activate any parameter changes on the hardware.
            |
            | Called after a param_tree_changed signal from DAQ_Move_main.
        # Unused
        """
        if param.name == 'max_cl_freq':
            channel = self.settings.child('multiaxes', 'axis').value()
            ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.MAX_CL_FREQUENCY,
                            self.settings.child("stage_parameters", 'max_cl_freq').value())

        elif param.name == 'hold_time':
            channel = self.settings.child('multiaxes', 'axis').value()
            ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.HOLD_TIME,
                                self.settings.child("stage_parameters", 'hold_time').value())

        elif param.name() == 'quiet_mode':
            channel = self.settings.child('multiaxes', 'axis').value()
            if param.value():
                ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.ACTUATOR_MODE, ctl.ActuatorMode.QUIET)
            else:
                ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.ACTUATOR_MODE, ctl.ActuatorMode.NORMAL)

        elif param.name() == 'move_velocity':
            ctlerr('SetProperty_i64', self.controller, self.settings.child('multiaxes', 'axis').value(), ctl.Property.MOVE_VELOCITY,
                   round(self.settings.child("stage_parameters", 'move_velocity').value() * 1e9))

        elif param.name() == 'move_acceleration':
            ctlerr('SetProperty_i64', self.controller, self.settings.child('multiaxes', 'axis').value(), ctl.Property.MOVE_ACCELERATION,
                   round(self.settings.child("stage_parameters", 'move_acceleration').value() * 1e9))

        elif param.name() == 'step_amplitude':
            ctlerr('SetProperty_i32', self.controller, self.settings.child('multiaxes', 'axis').value(),
                   ctl.Property.STEP_AMPLITUDE,
                   round(self.settings.child("stage_parameters", 'step_amplitude').value()))

        elif param.name() == 'step_frequency':
            ctlerr('SetProperty_i32', self.controller, self.settings.child('multiaxes', 'axis').value(),
                   ctl.Property.STEP_FREQUENCY,
                   round(self.settings.child("stage_parameters", 'step_frequency').value()))

    def move_abs(self, value):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """
        channel = self.settings.child('multiaxes', 'axis').value()

        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one

        if self.has_sensor:
            if self.move_mode == ctl.MoveMode.CL_RELATIVE:  # Switch to absolute
                self.move_mode = ctl.MoveMode.CL_ABSOLUTE
                ctlerr('SetProperty_i32',self.controller, channel, ctl.Property.MOVE_MODE, self.move_mode)
            value *= 1e6  # Back to picometers or nanodegrees
            # Move command
            ctlerr('Move', self.controller, channel, int(value), 0)

        # If in step mode, we manually update the position
        else:
            # In step mode we can only do relative
            rel_move = value - self.step_position
            self.move_rel(rel_move)

        self.emit_status(ThreadCommand('Update_Status', [f'Moving to {self.target_value}']))


    def move_rel(self, value):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        channel = self.settings.child('multiaxes', 'axis').value()
        cur_pos = self.get_actuator_value()

        value = self.check_bound(cur_pos + value) - cur_pos
        relative_move = self.set_position_relative_with_scaling(value)

        if self.has_sensor:     # If not, value will be understood as a number of steps
            if self.move_mode == ctl.MoveMode.CL_ABSOLUTE:  # Switch to relative
                self.move_mode = ctl.MoveMode.CL_RELATIVE
                ctlerr('SetProperty_i32', self.controller, channel, ctl.Property.MOVE_MODE, self.move_mode)
            value = relative_move * 1e6  # Back to picometers or nanodegrees

        # Move command
        ctlerr('Move', self.controller, channel, int(value), 0)

        # If in step mode, we manually update the position
        if not self.has_sensor:
            self.current_position += value
            self.step_position += value
            self.parent.check_position()

        self.emit_status(ThreadCommand('Update_Status', [f'Relative move by {relative_move}']))


    def move_home(self):
        """Move to the physical reference and reset position to 0
        """
        channel = self.settings.child('multiaxes', 'axis').value()
        ctlerr('Reference', self.controller, channel)
        self.emit_status(ThreadCommand('Update_Status',
                                       ['Referencing has been launched']))

        channel_state = ctlerr('GetProperty_i32', self.controller, channel, ctl.Property.CHANNEL_STATE)

        # wait for referencing to be done
        while channel_state & ctl.ChannelState.REFERENCING:
            time.sleep(1)
            channel_state = ctlerr('GetProperty_i32', self.controller, channel, ctl.Property.CHANNEL_STATE)

        self.parent.check_position()
        if channel_state & ctl.ChannelState.IS_REFERENCED:
            self.settings.child('stage_parameters', 'is_referenced').setValue(True)
        else:
            self.settings.child('stage_parameters', 'is_referenced').setValue(False)


    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""
        ctlerr('Stop', self.controller, self.settings.child('multiaxes', 'axis').value())
        self.emit_status(ThreadCommand('Update_Status',
                                       ['The positioner has been stopped']))


if __name__ == '__main__':
    main(__file__)
