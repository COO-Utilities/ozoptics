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

import errno
import logging
import time
import socket
import threading
import sys


class Controller:
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
    last_error = ""

    def __init__(self, log: bool =True, logfile: str =None):

        """
        Class to handle communications with the stage controller and any faults

        :param log: Boolean, whether to log to file or not
        :param logfile: Filename for log

        NOTE: default is INFO level logging, use set_verbose to increase verbosity.
        """

        # thread lock
        self.lock = threading.Lock()

        # Set up socket
        self.socket = None
        self.connected = False

        self.current_attenuation = None
        self.current_position = None
        self.configuration = ""
        self.homed = False

        # set up logging
        self.verbose = False
        if log:
            if logfile is None:
                logfile = __name__.rsplit('.', 1)[-1] + '.log'
            self.logger = logging.getLogger(logfile)
            self.logger.setLevel(logging.INFO)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            file_handler = logging.FileHandler(logfile)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

            console_formatter = logging.Formatter(
                '%(asctime)s--%(message)s')
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(console_formatter)
            self.logger.addHandler(console_handler)
        else:
            self.logger = None

    def set_verbose(self, verbose: bool =True):
        """ Set verbose mode.

        :param verbose: Boolean, set to True to enable DEBUG level messages,
                        False to disable DEBUG level messages
        """
        self.verbose = verbose
        if self.logger:
            if self.verbose:
                self.logger.setLevel(logging.DEBUG)
            else:
                self.logger.setLevel(logging.INFO)

    def connect(self, host: str =None, port: int =None):
        """ Connect to stage controller.

        :param host: String, host ip address
        :param port: Int, Port number
        """
        if self.socket is None:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((host, port))
            if self.logger:
                self.logger.debug("Connected to %(host)s:%(port)s", {
                    'host': host,
                    'port': port
                })
            self.connected = True

        except OSError as ex:
            if ex.errno == errno.EISCONN:
                if self.logger:
                    self.logger.debug("Already connected")
                self.connected = True
            else:
                if self.logger:
                    self.logger.error("Connection error: %s", ex.strerror)
                self.connected = False
        # clear socket
        if self.connected:
            self.__clear_socket()

    def disconnect(self):
        """ Disconnect stage controller. """
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()
            self.socket = None
            if self.logger:
                self.logger.debug("Disconnected controller")
            self.connected = False
        except OSError as ex:
            if self.logger:
                self.logger.error("Disconnection error: %s", ex.strerror)
            self.connected = False
            self.socket = None

    def __clear_socket(self):
        """ Clear socket buffer. """
        if self.socket is not None:
            self.socket.setblocking(False)
            while True:
                try:
                    _ = self.socket.recv(1024)
                except BlockingIOError:
                    break
            self.socket.setblocking(True)

    def __read_response(self):
        """Read the return message from stage controller."""
        # Get return value
        recv = self.socket.recv(2048)

        # Return type
        msg_type = 'data'

        # Did we get the entire return?
        tries = 5
        while tries > 0 and b'Done' not in recv:
            recv += self.socket.recv(2048)
            if b'Error' in recv:
                if self.logger:
                    self.logger.error(recv)
                return {'error': self.__return_parse_error(str(recv.decode('utf-8')))}
            tries -= 1

        recv_len = len(recv)
        if self.logger:
            self.logger.debug("Return: len = %d, Value = %s", recv_len, recv)

        if b'Done' not in recv:
            if self.logger:
                self.logger.warning("Read from controller timed out")
            msg_type = 'error'

        return {msg_type: str(recv.decode('utf-8'))}

    def __extract_position(self, retval):
        """ Read returned attenuator position in steps from home from controller """

        # Are we a valid return value?
        posn = None
        if 'Done' in retval and 'Pos:' in retval:
            if self.logger:
                self.logger.debug("Return value validated")
            posn = int(retval.split('Pos:')[-1].split()[0])
        else:
            self.logger.error("Return value not valid")
        return posn

    def __extract_attenuation(self, retval):
        """ Read returned attenuation in dB from controller """

        # Are we a valid return value?
        attn = None
        if 'Done' in retval and 'Attn:' in retval:
            if self.logger:
                self.logger.debug("Return value validated")
            attn = int(retval.split('Pos:')[-1].split()[0].split('(dB)')[0])
        else:
            self.logger.error("Return value not valid")
        return attn

    def __extract_configuration(self, retval):
        """ Read controller response """

        recv_len = len(retval)
        if 'Done' in retval and recv_len > 1:
            if self.logger:
                self.logger.debug("Return value validated")
        else:
            if self.logger:
                self.logger.warning("Return value error")

        return retval

    def __send_serial_command(self, cmd=''):
        """
        Send serial command to stage controller

        :param cmd: String, command to send to stage controller
        :return: dictionary {'data|error': string_message}
        """

        # Prep command
        cmd_send = f"{cmd}\r\n"
        if self.logger:
            self.logger.debug("Sending command:%s", cmd_send)
        cmd_encoded = cmd_send.encode('utf-8')

        # check connection
        if not self.connected:
            msg_text = "Not connected to controller!"
            if self.logger:
                self.logger.error(msg_text)

        try:
            self.socket.settimeout(30)
            # Send command
            self.socket.send(cmd_encoded)
            time.sleep(.05)
            msg_type = 'data'
            msg_text = 'Command sent successfully'

        except socket.error as ex:
            msg_type = 'error'
            msg_text = f"Command send error: {ex.strerror}"
            if self.logger:
                self.logger.error(msg_text)

        return {msg_type: msg_text}

    def __send_command(self, cmd="", parameters=None, custom_command=False):
        """
        Send a command to the stage controller

        :param cmd: String, command to send to the stage controller
        :param parameters: List of string parameters associated with cmd
        :param custom_command: Boolean, if true, command is custom
        :return: dictionary {'data|error': string_message}
        """

        # verify cmd and stage_id
        ret = self.__verify_send_command(cmd, custom_command)
        if 'error' in ret:
            return ret

        # Check if the command should have parameters
        if cmd in self.parameter_commands and parameters:
            if self.logger:
                self.logger.debug("Adding parameters")
            parameters = [str(x) for x in parameters]
            parameters = "".join(parameters)
            cmd += parameters

        if self.logger:
            self.logger.debug("Input command: %s", cmd)

        # Send serial command
        with self.lock:
            result = self.__send_serial_command(cmd)

        return result

    def __verify_send_command(self, cmd, custom_command=False):
        """ Verify cmd and stage_id

        :param cmd: String, command to send to the stage controller
        :param custom_command: Boolean, if true, command is custom
        :return: dictionary {'data|error': string_message}"""

        # Do we have a connection?
        if not self.connected:
            msg_type = 'error'
            msg_text = 'Not connected to controller'

        else:
            # Do we have a legal command?
            if cmd.rstrip().upper() in self.controller_commands:
                msg_type = 'data'
                msg_text = f"{cmd} is a valid or custom command"
            else:
                if not custom_command:
                    msg_type = 'error'
                    msg_text = f"{cmd} is not a valid command"
                else:
                    msg_type = 'data'
                    msg_text = f"{cmd} is a custom command"

        return {msg_type: msg_text}

    def __return_parse_error(self, error=""):
        """
        Parse the return error message from the controller.  The message code is
        given in the last string character

        :param error: Error code from the controller
        :return: String message
        """
        error = error.rstrip()
        return self.error.get(error, "Unknown error")

    def home(self):
        """
        Home the stage

        :return: return from __send_command
        """

        if not self.homed:
            ret = self.__send_command(cmd='H')

            if 'data' in ret:
                ret = self.__read_response()
                if 'error' in ret:
                    if self.logger:
                        self.logger.error(ret['error'])
                else:
                    if self.logger:
                        self.logger.debug(ret['data'])
            else:
                if self.logger:
                    self.logger.error(ret['error'])
        else:
            ret = {'data': 'already homed' }

        return ret

    def set_attenuation(self, atten=None):
        """
        Move stage to absolute position and return when in position

        :param atten: Float, absolute attenuation in fraction
        :return: dictionary {'data|error': current_attenuation|string_message}
        """

        # Send move to controller
        ret = self.__send_command(cmd="A", parameters=[atten])

        if 'data' in ret:
            retval = self.__read_response()
            cur_atten = self.__extract_attenuation(retval['data'])
            if cur_atten != atten:
                if self.logger:
                    self.logger.error("Attenuation setting not achieved!")
            self.current_attenuation = cur_atten
            return {'data': cur_atten}

        return ret

    def step(self, direction:str = 'F'):
        """
        Move stage to relative position and return when in position
        :param direction: String, 'F' - forward or 'B' - backward
        :return: dictionary {'data|error': current_position|string_message}
        """
        # check inputs
        if direction not in ['F', 'B']:
            if self.logger:
                self.logger.error("Invalid direction")
            return {'error': 'Invalid direction'}

        ret = self.__send_command(cmd=direction)

        if 'data' in ret:
            retval = self.__read_response()
            if 'data' in retval:
                ret = {}
                position = self.__extract_position(retval['data'])
                if position is not None:
                    self.current_position = position
                    ret['data'] = position
                else:
                    ret['error'] = "Invalid position"
            else:
                ret = retval
        return ret

    def get_position(self):
        """ Current position

        :return: dictionary {'data|error': current_position|string_message}
        """

        ret = self.__send_command(cmd="S?")
        if 'data' in ret:
            retval = self.__read_response()
            if 'data' in retval:
                ret = {}
                position = self.__extract_position(retval['data'])
                if position is not None:
                    self.current_position = position
                    ret['data'] = position
                else:
                    ret['error'] = "Invalid position"
            else:
                ret = retval
        return ret

    def reset(self):
        """ Reset stage

        :return: return from __send_command
        """

        ret = self.__send_command(cmd="RS")
        time.sleep(2.)

        if 'error' not in ret:
            self.read_from_controller()

        return ret

    def get_params(self):
        """ Get stage parameters

        :return: return from __send_command
        """

        ret = self.__send_command(cmd="CD")

        if 'data' in ret:
            retval = self.__read_response()
            if 'data' in retval:
                self.configuration = retval['data']
                ret['data'] = self.configuration
            else:
                ret = retval
        return ret

    def initialize_controller(self):
        """ Initialize stage controller. """
        ret = self.home()
        if 'data' in ret:
            retval = self.__read_response()
            if 'data' in retval:
                ret['data'] = 'intialized'
            else:
                ret = retval
        return ret

    def read_from_controller(self):
        """ Read from controller"""
        self.socket.setblocking(False)
        try:
            recv = self.socket.recv(2048)
            recv_len = len(recv)
            if self.logger:
                self.logger.debug("Return: len = %d, Value = %s", recv_len, recv)
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

            ret = self.__send_command(cmd=cmd, custom_command=True)
            if 'error' not in ret:
                output = self.read_from_controller()
                print(output)

            if self.logger:
                self.logger.debug("End: %s", ret)
