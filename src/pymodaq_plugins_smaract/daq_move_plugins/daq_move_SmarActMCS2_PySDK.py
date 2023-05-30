from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main  # common set of parameters for all actuators
from pymodaq.daq_utils.daq_utils import ThreadCommand, getLineInfo  # object used to send info back to the main thread
from pymodaq.daq_utils.parameter import Parameter
from pymodaq.daq_utils.config import Config
config = Config()

try:
    import smaract.ctl as ctl
except ImportError:
    raise RuntimeError("The Smaract Python SDK was not found.")

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

version = ctl.GetFullVersionString()
assert_lib_compatibility()

buffer = ctl.FindDevices()
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
                      {'title': 'Max. Closed Loop Frequency:',
                       'name': 'max_cl_freq',
                       'type': int,
                       'value': 6000},
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
                                new_controller=ctl.Open(self.settings.child('controller_parameters',
                                                                            'controller_locator').value()))
            channel = self.settings.child('multiaxes', 'axis').value()
            type = ctl.GetProperty_i32(self.controller, channel, ctl.Property.CHANNEL_TYPE)

            if type == ctl.ChannelModuleType.STICK_SLIP_PIEZO_DRIVER:
                # Set max closed loop frequency (maxCLF) to 6 kHz. This properties sets a limit for the maximum actuator driving frequency.
                # The maxCLF is not persistent thus set to a default value at startup.
                ctl.SetProperty_i32(self.controller, channel, ctl.Property.MAX_CL_FREQUENCY, 6000)
                # The hold time specifies how long the position is actively held after reaching the target.
                # This property is also not persistent and set to zero by default.
                # A value of 0 deactivates the hold time feature, the constant ctl.HOLD_TIME_INFINITE sets the time to infinite.
                # (Until manually stopped by "Stop") Here we set the hold time to 1000 ms.
                ctl.SetProperty_i32(self.controller, channel, ctl.Property.HOLD_TIME, 1000)
            elif type == ctl.ChannelModuleType.MAGNETIC_DRIVER:
                # Enable the amplifier (and start the phasing sequence).
                ctl.SetProperty_i32(self.controller, channel, ctl.Property.AMPLIFIER_ENABLED, ctl.TRUE)

            self.has_sensor = ctl.ChannelState.SENSOR_PRESENT

            initialized = True
        except:
            initialized = False

        info = "Smaract stage initialized"
        return info, initialized


    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        channel = self.settings.child('multiaxes', 'axis').value()
        pos = ctl.GetProperty_i64(self.controller, channel, ctl.Property.POSITION)
        pos = float(pos) / 1e6  # the position given by the
        # controller is in picometers, we convert in micrometers
        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Close the communication with the MCS2 controller.
        """
        self.controller.close_communication()
        self.controller = None

    def commit_settings(self, param: Parameter):
        """
            | Activate any parameter changes on the hardware.
            |
            | Called after a param_tree_changed signal from DAQ_Move_main.
        # Unused

        """

    def move_abs(self, value):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        value = int(value * 1e6)
        self.controller.absolute_move(
            self.settings.child('multiaxes', 'axis').value(), value)
        self.emit_status(ThreadCommand('Update_Status', [f'Moving to {self.target_value}']))

    def move_rel(self, value):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        relative_move = self.set_position_relative_with_scaling(value)

        # convert relative_move in picometers
        relative_move = int(relative_move*1e6)

        self.controller.relative_move(
            self.settings.child('multiaxes', 'axis').value(),
            relative_move)
        self.emit_status(ThreadCommand('Update_Status', [f'Moving to {self.target_value}']))

    def move_home(self):
        """Move to the physical reference and reset position to 0
        """
        self.controller.find_reference(
            self.settings.child('multiaxes', 'axis').value())
        self.emit_status(ThreadCommand('Update_Status',
                                       ['The positioner has been referenced']))

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""
        self.controller.stop(self.settings.child('multiaxes', 'axis').value())
        self.emit_status(ThreadCommand('Update_Status',
                                       ['The positioner has been stopped']))


if __name__ == '__main__':
    main(__file__)