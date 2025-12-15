# coding=utf-8
"""
The following controller commands are available.  Not that many are not implemented at the moment.
The asterisk indicates the commands that are implemented.

A<n>      * - Sets attenuation to <n>.
        Two digits to the right of the decimal point are allowed, but not required.
A?        * - Gets attenuation.
B         * - Steps the attenuator one step backward.
        Returns the final position after the command is completed.
CD          - Sends the current unit configuration only to the RS-232 communications port.
CH<hh>      - Sets the I2C address to the hexadecimal address <hh>
        when the address <hh> is a valid I2C address between 0x00 and 0x7F.
CI<ddd>     - Sets the I2C address to the decimal address <ddd>
        when the address <ddd> is a valid I2C address between 0 and 127.
CS<cp><dp>  - Sets the SPI parameters to <cp> and <dp>,
        where <cp> is clock polarity and <dp> is data position.
D         * - Gets current attenuation and step position.
E0        * - In RS-232 mode, sets echo to OFF.
        The unit does not echo any characters received through the RS-232 interface.
E1        * - In RS-232 mode, sets echo to ON.
        The unit echoes all characters received through the RS-232 interface.
EVA8        - Set the unit in EVA8 mode
EVA9        - Set the unit in EVA9 mode
EVA?        - Requests the current EVA configuration mode
F         * - Steps the attenuator one step forward.
H         * - Re-homes the unit.
I2C?        - Gets I2C/SPI bus  Voltage
I2C3        - Sets I2C/SPI bus voltage to 3.3V
I2C5        - Sets I2C/SPI bus voltage to 5.0V
L<n>        - Sets the unit’s insertion loss to <n>.
        Two digits following the decimal point are allowed, but not required.
RES?      * - Read previous command response
RST       * - Restarts in response to a hardware or software reset (RST) command
        and is in self-test mode.
S?        * - Requests the current position of the attenuator.
        Returns the current number of steps from the home position.
S<n>      * - Sets the position of the attenuator to <n> steps from the home position.
S+<n>     * - Sets the step position of the attenuator to <n> steps
        numerically greater than the current position.
S-<n>     * - Sets the step position of the attenuator to <n> steps
        numerically less than the current position.
W<n>        - Selects the wavelength using <n>.
    This command is valid only when the unit is calibrated for more than one wavelength.

"""
import dataclasses
import enum
import errno
import time
import socket
from typing import Union, Dict, Tuple

from hardware_device_base import HardwareMotionBase

class ResponseType(enum.Enum):
    """Controller response types."""
    ATTEN = "attenuation"
    POS = "steps"
    DIFF = "diff"
    BOTH = "attenuation and steps"
    STRING = "string"
    ERROR = "error"


@dataclasses.dataclass
class OzResponse:
    """Oz controller response data."""
    type: ResponseType
    value: Union[float, int, str, dict, None]


class OZController(HardwareMotionBase):
    """
    Controller class for OZ Optics DD-100-MC Attenuator Controller.
    """
    # pylint: disable=too-many-instance-attributes

    controller_commands = ["A",     # Set attenuation
                           "A?",    # Get attenuation
                           "B",     # Move attenuator one step backward
                           "CD",    # Configuration Display
                           "D",     # Gets current attenuation and step position
                           "E0",    # In RS232 mode, sets echo to OFF
                           "E1",    # In RS232 mode, sets echo to ON
                           "F",     # Move attenuator one step forward
                           "H",     # Re-homes the unit
                           "L",     # Insertion loss
                           "RES?",  # Read previous command response
                           "RST",   # Restarts in self-test mode
                           "S?",    # Requests current position of the attenuator
                           "S",     # Sets the position of the attenuator to <n> steps from home
                           "S+",    # Adds <n> steps to current position
                           "S-"     # Subtracts <n> steps from current position
                           ]
    return_value_commands = ["A", "A?", "B", "CD", "D", "F", "H", "L",
                             "RES?", "RST", "S?", "S", "S+", "S-" ]
    parameter_commands = ["A", "L", "S", "S+", "S-"]
    error = {
        "Done": "No error.",
        "Error-2": "Bad command.  The command is ignored.",
        "Error-5": "Home sensor error.  Return unit to factory for repair.",
        "Error-6": "Overflow.  The command is ignored.",
        "Error-7": "Motor voltage exceeds safe limits"
    }

    def __init__(self, log: bool =True, logfile: str =__name__.rsplit(".", 1)[-1]):

        """
        Class to handle communications with the stage controller and any faults

        :param log: Boolean, whether to log to file or not
        :param logfile: Filename for log

        NOTE: default is INFO level logging, use set_verbose to increase verbosity.
        """
        super().__init__(log, logfile)

        # Set up socket
        self.socket = None

        self.current_attenuation = None
        self.current_position = None
        self.current_diff = None
        self.configuration = ""
        self.homed = False
        self.last_error = ""

    def _clear_socket(self):
        """ Clear socket buffer. """
        if self.socket is not None:
            self.socket.setblocking(False)
            while True:
                try:
                    _ = self.socket.recv(1024)
                except BlockingIOError:
                    break
            self.socket.setblocking(True)

    def _read_reply(self) -> Union[OzResponse, None]:
        """Read the return message from stage controller."""
        # Get return value
        recv = self.socket.recv(2048)

        # Did we get the entire return?
        tries = 5
        while tries > 0 and b'Done' not in recv:
            recv += self.socket.recv(2048)
            if b'Error' in recv:
                self.report_error(recv.decode('utf-8'))
                error_string = self._return_parse_error(str(recv.decode('utf-8')))
                return OzResponse(ResponseType.ERROR, error_string)
            tries -= 1

        recv_len = len(recv)
        self.report_debug(f"Return: len = {recv_len}, Value = {recv}")

        if b'Done' not in recv:
            msg_data = str(recv.decode('utf-8'))
            self.report_error(f"Read from controller timed out: {msg_data}")
            return None

        resp = self._parse_response(str(recv.decode('utf-8')))
        if resp.type == ResponseType.ERROR:
            self.report_error(resp.value)

        return resp

    def _parse_response(self, raw: str) -> OzResponse:
        """Parse the response from stage controller."""
        # pylint: disable=too-many-branches
        raw = raw.strip()

        if 'Pos:' in raw:
            try:
                pos = int(raw.split('Pos:')[1].split()[0])
                self.current_position = pos
                pos_read = True
            except ValueError:
                self.report_error("Error parsing position")
                pos = None
                pos_read = False
        else:
            pos = None
            pos_read = False

        if 'Atten:' in raw:
            try:
                if 'unknown' in raw:
                    atten = None
                else:
                    atten = float(raw.split('Atten:')[1].split('(')[0])
                self.current_attenuation = atten
                atten_read = True
            except ValueError:
                self.report_error("Error parsing attenuation")
                atten = None
                atten_read = False
        else:
            atten = None
            atten_read = False

        # Diff (after homing)
        if 'Diff=' in raw:
            try:
                diff = float(raw.split('Diff=')[1].split()[0])
                self.current_diff = diff
                self.current_position = 0
                diff_read = True
            except ValueError:
                self.report_error("Error parsing diff")
                diff = None
                diff_read = False
        else:
            diff = None
            diff_read = False

        # Error cases
        if 'Error' in raw or self.status < 0:
            error_string = raw if self.status >= 0 else self.status_string
            return OzResponse(ResponseType.ERROR, error_string)

        # Both Attenuation and Steps
        if pos_read and atten_read:
            return OzResponse(ResponseType.BOTH, {"pos": pos, "atten": atten})

        # Attenuation
        if atten_read:
            return OzResponse(ResponseType.ATTEN, atten)

        # Pos
        if pos_read:
            return OzResponse(ResponseType.POS, pos)

        # Diff (after homing)
        if diff_read:
            return OzResponse(ResponseType.DIFF, diff)

        # Default to string
        return OzResponse(ResponseType.STRING, raw)

    def _send_serial_command(self, cmd='') -> bool:
        """
        Send serial command to stage controller

        :param cmd: String, command to send to stage controller
        :return: dictionary {'data|error': string_message}
        """

        # check connection
        if not self.connected:
            self.report_error("Not connected to controller!")
            return False

        # Prep command
        cmd_send = f"{cmd}\r\n"
        self.report_debug(f"Sending command: {cmd_send}")
        cmd_encoded = cmd_send.encode('utf-8')

        try:
            self.socket.settimeout(30)
            # Send command
            self.socket.send(cmd_encoded)
            time.sleep(.05)
            return True

        except socket.error as ex:
            self.report_error(f"Command send error: {ex.strerror}")
            return False

    def _send_command(self, command: str, *args, custom_command=False) -> bool:
        # pylint: disable=W0221
        """
        Send a command to the stage controller

        :param command: String, command to send to the stage controller
        :param *args: List of string parameters associated with cmd
        :param custom_command: Boolean, if true, command is custom
        :return: dictionary {'data|error': string_message}
        """

        # verify cmd and stage_id
        if not self._verify_send_command(command, custom_command):
            return False

        # Check if the command should have parameters
        if command in self.parameter_commands and args:
            self.report_debug("Adding parameters")
            parameters = [str(x) for x in args]
            parameters = "".join(parameters)
            command += parameters

        self.report_debug(f"Input command: {command}")

        # Send serial command
        with self.lock:
            result = self._send_serial_command(command)

        return result

    def _verify_send_command(self, cmd, custom_command=False) -> bool:
        """ Verify cmd and stage_id

        :param cmd: String, command to send to the stage controller
        :param custom_command: Boolean, if true, command is custom
        :return: dictionary {'data|error': string_message}"""

        # Do we have a connection?
        if not self.is_connected():
            self.report_error('Not connected to controller')
            return False

        # Do we have a legal command?
        if cmd.rstrip().upper() in self.controller_commands:
            self.report_info(f"{cmd} is a valid command")
            return True
        if not custom_command:
            self.report_error(f"{cmd} is not a valid command")
            return False
        self.report_info(f"{cmd} is a custom command")
        return True

    def _return_parse_error(self, error=""):
        """
        Parse the return error message from the controller.  The message code is
        given in the last string character

        :param error: Error code from the controller
        :return: String message
        """
        error = error.rstrip()
        return self.error.get(error, "Unknown error")

    # --- User-Facing Methods
    def connect(self, host, port,  con_type: str="tcp") -> None:  # pylint: disable=W0221
        """ Connect to stage controller.

        :param host: String, for tcp connection, host (name or IP)
        :param port: Int, for tcp connection, port
        :param con_type: String, tcp or serial (tcp only supported)
        """
        if self.validate_connection_params((host, port)):
            if con_type == "tcp":
                if self.socket is None:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    self.socket.connect((host, port))
                    self.report_info(f"Connected to {host}:{port}")
                    self._set_connected(True)

                except OSError as ex:
                    if ex.errno == errno.EISCONN:
                        self.report_info("Already connected")
                        self._set_connected(True)
                    else:
                        self.report_error(f"Connection error: {ex.strerror}")
                        self._set_connected(False)
                # clear socket
                if self.is_connected():
                    self._clear_socket()
            elif con_type == "serial":
                self.report_error("Serial connection not implemented")
                self._set_connected(False)
            else:
                self.report_error(f"Unknown con_type: {con_type}")
                self._set_connected(False)
        else:
            self.report_error(f"Invalid connection args: {host}:{port}")
            self._set_connected(False)

    def disconnect(self):
        """ Disconnect stage controller. """
        if not self.is_connected():
            self.report_warning("Already disconnected from device")
            return
        try:
            self.report_info("Disconnecting from stage controller")
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()
            self.socket = None
            self._set_connected(False)
            self.report_info("Disconnected from stage controller")
        except OSError as ex:
            self.report_error(f"Disconnection error: {ex.strerror}")
            self._set_connected(False)
            self.socket = None

    def home(self) -> bool:
        """
        Home the stage

        :return: True if home was successful, False otherwise
        """
        if not self.is_homed():
            if self._send_command('H'):
                self.current_attenuation = None
                self.current_position = None
                resp = self._read_reply()
                if resp.type == ResponseType.DIFF:
                    self.homed = True
                    self.report_debug(f"{resp.value}")
                elif resp.type == ResponseType.ERROR:
                    self.report_error(f"{resp.value}")
        else:
            self.report_warning("Already homed.")

        return self.homed

    def is_homed(self) -> bool:
        """ Has the stage controller been homed?"""
        return self.homed

    def get_atomic_value(self, item: str ="") -> Union[float, int, str, None]:
        """Return single value for item"""
        if "pos" in item:
            result = self.get_pos()
            if result is None:
                self.report_error("Failed to get position")
                value = None
            else:
                value = int(result)
        elif "atten" in item:
            result = self.get_attenuation()
            if result is None:
                self.report_error("Failed to get attenuation")
                value = None
            else:
                value = float(result)
        else:
            self.report_error(f"Unknown item: {item}, choose pos or atten")
            value = None
        return value

    def set_attenuation(self, atten: float=None) -> bool:
        """
        Move stage to input attenuation and return when in position

        :param atten: Float, absolute attenuation in dB (0. - 60.)
        :return: True if successful, False otherwise
        """
        # check attenuation limits
        if atten is None or atten < 0.0 or atten > 60.0:
            self.report_error(f"Invalid attenuation: {atten}, cannot be < 0. or > 60.")
            return False

        # Send move to controller
        if self._send_command("A", atten):
            resp = self._read_reply()
            if resp.type == ResponseType.POS:
                time.sleep(0.5)
                cur_atten = self.get_attenuation()
                self.report_debug(f"{cur_atten}")
                if cur_atten != atten:
                    self.report_error("Attenuation setting not achieved!")
                    return False
                return True
            if resp.type == ResponseType.ERROR:
                self.report_error(f"{resp.value}")
            else:
                self.report_error("Attenuation setting not achieved!")
            return False
        self.report_error("Attenuation setting not achieved!")
        return False

    def set_pos(self, pos=None):  # pylint: disable=W0221
        """
        Move stage to absolute position and return when in position

        :param pos: Int, absolute position in steps
        :return: dictionary {'data|error': current_attenuation|string_message}
        """

        # Send move to controller
        if self._send_command("S", pos):
            resp = self._read_reply()
            if resp.type == ResponseType.POS:
                time.sleep(0.5)
                cur_pos = resp.value
                self.report_debug(f"{cur_pos}")
                if cur_pos != pos:
                    self.report_error("Position setting not achieved!")
                    return False
                self.get_attenuation()
                return True
            if resp.type == ResponseType.ERROR:
                self.report_error(f"{resp.value}")
            else:
                self.report_error("Position setting not achieved!")
            return False
        self.report_error("Position setting not achieved!")
        return False

    def step(self, direction:str = 'F') -> Union[int, None]:
        """
        Move stage to relative position and return when in position
        :param direction: String, 'F' - forward or 'B' - backward
        :return: Current position in steps or None
        """
        direc = direction.upper()
        # check inputs
        if direc not in ['F', 'B']:
            self.report_error("Invalid direction: use F or B")
            return None

        if self._send_command(direc):
            resp = self._read_reply()
            if resp.type == ResponseType.POS:
                self.report_debug(f"{resp.value}")
                cur_pos = resp.value
                if cur_pos != self.current_position:
                    self.report_error("Position setting not achieved!")
                    self.current_position = cur_pos
                return cur_pos
            if resp.type == ResponseType.ERROR:
                self.report_error(f"{resp.value}")
            else:
                self.report_error("Position setting not achieved!")
            return None
        self.report_error("Position setting not achieved!")
        return None

    def get_pos(self) -> Union[int, None]:  # pylint: disable=W0221
        """ Current position

        :return: current position in steps or None
        """

        if self._send_command("S?"):
            resp = self._read_reply()
            if resp.type == ResponseType.POS:
                self.report_debug(f"{resp.value}")
                return resp.value
            if resp.type == ResponseType.ERROR:
                self.report_error(f"{resp.value}")
            return None
        return None

    def get_attenuation(self) -> Union[float, None]:
        """ Current attenuation

        :return: dictionary {'data|error': current_attenuation|string_message}
        """

        if self._send_command("A?"):
            resp = self._read_reply()
            if resp.type == ResponseType.ATTEN:
                self.report_debug(f"{resp.value}")
                return resp.value
            if resp.type == ResponseType.ERROR:
                self.report_error(f"{resp.value}")
            return None
        return None

    def reset(self):
        """ Reset stage

        :return: Configuration string
        """

        if self._send_command("RST"):
            time.sleep(2.)
            resp = self._read_reply()
            if resp.type == ResponseType.STRING:
                self.report_debug(f"{resp.value}")
                return resp.value
            if resp.type == ResponseType.ERROR:
                self.report_error(f"{resp.value}")
            return None
        self.report_error("Failed to reset stage")
        return None

    def get_params(self) -> Union[str, None]:
        """ Get stage parameters

        :return: return from __send_command
        """

        if self._send_command("CD"):
            resp = self._read_reply()
            if resp.type == ResponseType.STRING:
                self.report_debug(f"{resp.value}")
                self.configuration = resp.value
                return resp.value
            if resp.type == ResponseType.ERROR:
                self.report_error(f"{resp.value}")
            return None
        self.report_error("Failed to get stage parameters")
        return None

    def initialize(self) -> bool:
        """ Initialize stage controller. """
        if not self.home():
            self.report_error("Failed to initialize controller")
        return self.homed

    def read_from_controller(self) -> str:
        """ Read from controller"""
        self.socket.setblocking(False)
        try:
            recv = self.socket.recv(2048)
            recv_len = len(recv)
            self.report_debug(f"Return: len = {recv_len}, Value = {recv}")
        except BlockingIOError:
            recv = b""
        self.socket.setblocking(True)
        return str(recv.decode('utf-8'))

    def run_manually(self):
        """ Input stage commands manually

        :return: None
        """

        while True:

            cmd = input("Enter Command")

            if not cmd:
                break

            ret = self._send_command(cmd, custom_command=True)
            if 'error' not in ret:
                output = self.read_from_controller()
                self.report_info(output)

            self.report_info(f"End: {ret}")

    def close_loop(self) -> bool:
        """ Close loop"""
        return True

    def is_loop_closed(self) -> bool:
        """ Check if loop is closed"""
        return True

    def get_limits(self) -> Union[Dict[str, Tuple[float, float]], None]:
        """ Get stage limits"""
        return None
