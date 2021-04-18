from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals

import serial
import time
import re
import binascii
import struct


class Firmware(object):
    def __init__(self, path):
        self.ranges = dict()
        self.upper_address = 0x0000
        self.banking_detected = False
        self.sanity_check = dict()

        with open(path, "r") as f:
            for line in f:
                self._parseline(line)

    def _parseline(self, line):
        if line[0] != ":":
            return
        hexstr = line.rstrip()[1:-2]
        binstr = binascii.unhexlify(hexstr)
        command = binstr[3]
        lower_address = (binstr[1] << 8) + binstr[2]

        if (command == 4 and binstr[0] == 0x02 and (lower_address == 0x0000)):
            self.upper_address = (binstr[4] << 8) + binstr[5]
            self.banking_detected = True
        elif command == 0:
            address = lower_address + (self.upper_address << 16)
            bytes = bytearray(binstr[4:])
            if self.upper_address in self.sanity_check:
                self.sanity_check[self.upper_address][0] += len(bytes)
                if self.sanity_check[self.upper_address][1] > lower_address:
                    self.sanity_check[self.upper_address][1] = lower_address
                if self.sanity_check[
                        self.upper_address][2] < lower_address + len(bytes):
                    self.sanity_check[
                        self.upper_address][2] = lower_address + len(bytes)
            else:
                tmp_list = [
                    len(bytes), lower_address, lower_address + len(bytes)
                ]
                self.sanity_check[self.upper_address] = tmp_list
            self._insert(address, bytes)

    def _insert(self, address, bytes):
        candidate = address + len(bytes)
        if candidate in self.ranges:
            nextbytes = self.ranges.pop(candidate)
            bytes.extend(nextbytes)

        for candidate in self.ranges:
            prev_len = len(self.ranges[candidate])
            if (candidate + prev_len) == address:
                self.ranges[candidate].extend(bytes)
                return
        self.ranges[address] = bytes

    def code(self):
        return self.ranges


class FirmwareUploader(object):
    NOP = 0x00
    OK = 0x10
    FAILED = 0x11
    INSYNC = 0x12
    EOC = 0x20
    GET_SYNC = 0x21
    GET_DEVICE = 0x22
    CHIP_ERASE = 0x23
    LOAD_ADDRESS = 0x24
    PROG_FLASH = 0x25
    READ_FLASH = 0x26
    PROG_MULTI = 0x27
    READ_MULTI = 0x28
    PARAM_ERASE = 0x29
    REBOOT = 0x30

    PROG_MULTI_MAX = 32  # 64 causes serial hangs with some USB-serial adapters
    READ_MULTI_MAX = 255
    BANK_PROGRAMMING = -1

    def __init__(self, port=None, at_baud=57600, bootloader_baud=115200):
        self.port = port
        self.at_baud = at_baud
        self.bootloader_baud = bootloader_baud
        self.program_progress_callback = None
        self.verify_progress_callback = None

    def set_program_progress_callback(self, fun):
        self.program_progress_callback = fun

    def set_verify_progress_callback(self, fun):
        self.verify_progress_callback = fun

    def _read(self):
        c = self.port.read()
        if len(c) < 1:
            raise RuntimeError("Read timeout.")
        return int(c[0])

    def send(self, data):
        try:
            data = bytearray([data])
        except TypeError:
            data = bytearray(data)
        print("Sending: {}".format(data))
        self.port.write(data)

    def _get_sync(self):
        c = self._read()
        if c != self.INSYNC:
            txt = ("Sync failed. Expected INSYNC=0x{:02X} but got "
                   "0x{:02X}.".format(self.INSYNC, c))
            raise RuntimeError(txt)
        print("insync")
        c = self._read()
        if c != self.OK:
            raise RuntimeError("Sync failed. Expected OK=0x{:02X} but got "
                               "0x{:02X}.".format(self.OK, c))
        print("ok")

    def _sync(self):
        self.send([self.NOP] * (self.PROG_MULTI_MAX * 2))
        self.port.flush()
        self.port.reset_input_buffer()
        self.send([self.GET_SYNC, self.EOC])
        self.port.flush()
        self._get_sync()

    def _erase(self, erase_params=False):
        self.send([self.CHIP_ERASE, self.EOC])
        self._get_sync()
        if erase_params:
            self.send([self.PARAM_ERASE, self.EOC])
            self._get_sync()

    def _set_address(self, address, banking):
        print("Adress: {} {}".format(address & 0xFF, (address >> 8) & 0xFF))
        data = bytearray()
        if banking:
            if self.BANK_PROGRAMING != (address >> 16):
                self.BANK_PROGRAMMING = address >> 16
                if self.BANK_PROGRAMING == 0:
                    print("HOME")
                else:
                    print("BANK", self.BANK_PROGRAMMING)
            data = [
                self.LOAD_ADDRESS, address & 0xFF, (address >> 8) & 0xFF,
                (address >> 16) & 0xFF, self.EOC
            ]
            self.send(data)
        else:
            data = [
                self.LOAD_ADDRESS, address & 0xFF, (address >> 8) & 0xFF,
                self.EOC
            ]
            self.send(data)
        self._get_sync()

    def _program_single(self, data):
        self.send([self.PROG_FLASH] + list(data) + [self.EOC])
        self.__get_sync()

    def _program_multi(self, data):
        sync_count = 0
        while len(data):
            print("prog_multi: {}".format(sync_count))
            n = min(len(data), self.PROG_MULTI_MAX)
            block = data[:n]
            block = data[:n]
            data = data[n:]
            self.send([self.PROG_MULTI, n] + list(block) + [self.EOC])
            sync_count += 1
        for _ in range(sync_count):
            self._get_sync()

    def _verify_byte(self, data):
        self.send([self.READ_FLASH, self.EOC])
        if self._read() != data:
            return False
        self._get_sync()
        return True

    def _verify_multi(self, data):
        self.send([self.READ_MULTI, len(data), self.EOC])
        for i in data:
            if self._read() != i:
                return False
        self._get_sync()
        return True

    def _reboot(self):
        self.send(self.REBOOT)

    def _split(self, seq, length):
        return [seq[i:i + length] for i in range(0, len(seq), length)]

    def total_size(self, code):
        total = 0
        for address in code.keys():
            total += len(code[address])
        return total

    def _program(self, firmware):
        code = firmware.code()
        count = 0
        total = self.total_size(code)
        for address in sorted(list(code.keys())):
            self._set_address(address, firmware.banking_detected)
            groups = self._split(code[address], self.PROG_MULTI_MAX)
            for bytes in groups:
                self._program_multi(bytes)
                count += len(bytes)
                try:
                    self.program_progress_callback(count, total)
                except TypeError:
                    pass

    def _verify(self, firmware):
        code = firmware.code()
        count = 0
        total = self.total_size(code)
        for address in sorted(code.keys()):
            self._set_address(address, firmware.banking_detected)
            groups = self._split(code[address], self.READ_MULTI_MAX)
            for bytes in groups:
                if not self._verify_multi(bytes):
                    raise RuntimeError(
                        "Verification failed at 0x{:02X}".format(address))
                count += len(bytes)
                try:
                    self.verify_progress_callback(count, total)
                except TypeError:
                    pass

    def expect(self, pattern, timeout):
        pattern = re.compile(pattern)
        start = time.time()
        s = ""
        while time.time() < start + timeout:
            b = self.port.read().decode("utf-8")
            if b:
                s += str(b)
                if pattern.search(s) is not None:
                    return True
            else:
                time.sleep(0.01)
        return False

    def check_bootloader(self):
        for i in range(3):
            try:
                self._sync()
            except RuntimeError:
                pass
            else:
                return True
        return False

    def identify(self):
        self.send([self.GET_DEVICE, self.EOC])
        board_id = self._read()
        board_freq = self._read()
        self._get_sync()
        return board_id, board_freq

    def upload(self, firmware, erase_params=False):
        self.port.baudrate = self.bootloader_baud
        self.port.reset_input_buffer()
        self.port.flush()
        print("Checking bootloader")
        if not self.check_bootloader():
            raise RuntimeError("Failed to contact bootloader")
        print("Identify board")
        board_id, board_freq = self.identify()
        if firmware.banking_detected and not (board_id & 0x80):
            raise RuntimeError("This firmware requires a CPU with banking.")
        if (board_id & 0x80):
            firmware.banking_detected = True
        print("Erasing")
        self._erase(erase_params=erase_params)
        print("Programming")
        self._program(firmware)
        print("Verifying")
        self._verify(firmware)
        print("Rebooting")
        self._reboot()


class Configurator(object):

    EEPROM_PARAMETERS = dict(
        format=dict(register=0, readonly=True, options=None, default=None),
        serial_speed=dict(register=1,
                          readonly=False,
                          options=[2, 4, 9, 19, 38, 57, 115],
                          default=57),
        air_speed=dict(
            register=2,
            readonly=False,
            options=[2, 4, 8, 16, 19, 24, 32, 48, 64, 96, 128, 192, 250],
            default=64),
        netid=dict(register=3,
                   readonly=False,
                   options=list(range(0, 26)),
                   default=25),
        txpower=dict(register=4,
                     readonly=False,
                     options=[1, 2, 5, 8, 11, 14, 17, 20],
                     default=11),
        ecc=dict(register=5, readonly=False, options=[0, 1], default=0),
        mavlink=dict(register=6, readonly=False, options=[0, 1, 2], default=1),
        oppresend=dict(register=7, readonly=False, options=[0, 1], default=0),
        min_freq=dict(register=8,
                      readonly=False,
                      options=[433050],
                      default=433050),
        max_freq=dict(register=9,
                      readonly=False,
                      options=[434790],
                      default=434790),
        num_channels=dict(register=10,
                          readonly=False,
                          options=list(range(5, 51)),
                          default=20),
        duty_cycle=dict(register=11,
                        readonly=False,
                        options=list(range(10, 101)),
                        default=100),
        lbt_rssi=dict(register=12,
                      readonly=False,
                      options=list(range(0, 256)),
                      default=0),
        manchester=dict(register=13, readonly=False, options=[0, 1], default=0),
        rtscts=dict(register=14, readonly=False, options=[0, 1], default=0),
        nodeid=dict(register=15,
                    readonly=False,
                    options=list(range(0, 30)),
                    default=2),
        nodedestination=dict(register=16,
                             readonly=False,
                             options=list(range(0, 30)) + [65535],
                             default=65535),
        syncany=dict(register=17, readonly=False, options=[0, 1], default=0),
        nodecount=dict(register=18,
                       readonly=False,
                       options=list(range(2, 31)),
                       default=5),
    )

    AT_COMMAND = dict(write_eeprom="AT&W\r\n",
                      exit="ATO\r\n",
                      reboot="ATZ\r\n",
                      show_eeprom="ATI5\r\n",
                      show_version="ATI\r\n",
                      update="AT&UPDATE\r\n")

    def __init__(self, port=None, baud=57600):
        if port:
            self.port = serial.Serial(port, baud)
        else:
            self.port = serial.Serial()
        self.port.timeout = 0.2
        self._desired_params = dict()
        self._current_params = dict()

    def upload_firmware(self,
                        path,
                        program_progress_cb=None,
                        verify_progress_cb=None):
        uploader = FirmwareUploader(port=self.port)
        uploader.set_program_progress_callback(program_progress_cb)
        uploader.set_verify_progress_callback(verify_progress_cb)
        baud = self.port.baudrate
        timeout = self.port.timeout
        self.port.timeout = 3.0
        firmware = Firmware(path)
        self.port.write(self.AT_COMMAND["update"].encode())
        time.sleep(0.7)

        success = False
        try:
            print("hello")
            uploader.upload(firmware)
        except RuntimeError as e:
            print("{}".format(e))
            raise e
            success = False
        else:
            success = True
        finally:
            self.port.baudrate = baud
            self.port.timeout = timeout
        return success

    @staticmethod
    def get_default_params(writable_only=True):
        params = dict()
        for param in Configurator.EEPROM_PARAMETERS:
            if Configurator.EEPROM_PARAMETERS[param]["readonly"]:
                continue
            params[param] = Configurator.EEPROM_PARAMETERS[param]["default"]
        return params

    def _parse_ok(self, bytes):
        if len(bytes) < 4:
            return False
        if b"OK" in bytes[-4:]:
            return True
        return False

    def read_until_ok(self, tries=3):
        success = False
        while tries > 0 and not success:
            bytes = self.port.readline()
            print(bytes)
            if len(bytes) > 0:
                success = self._parse_ok(bytes)
            else:
                tries -= 1
        return success

    def set_desired_params(self, params):
        new_params = dict()
        for key in params:
            if key not in self.EEPROM_PARAMETERS:
                print("Skipping unknown paramter '{}'".format(key))
                continue
            new_params[key] = params[key]
        self._desired_params = new_params

    def write_params(self, params=None):
        failed_params = []
        if params is None:
            params = self._desired_params
        if self._read_param_format() == 27:
            current_nodecount = self._current_params["nodecount"]
            current_nodeid = self._current_params["nodeid"]
            desired_nodecount = params["nodecount"]
            desired_nodeid = params["nodeid"]
            if current_nodeid >= desired_nodecount:
                self._write_param(self.EEPROM_PARAMETERS["nodeid"]["register"],
                                  desired_nodeid)
            if desired_nodeid >= current_nodecount:
                self._write_param(
                    self.EEPROM_PARAMETERS["nodecount"]["register"],
                    desired_nodecount)
        for param in params:
            if self.EEPROM_PARAMETERS[param]["readonly"]:
                continue
            register = self.EEPROM_PARAMETERS[param]["register"]
            if not (self._write_param(register, params[param])):
                failed_params.append(register)
        return failed_params

    def write_eeprom(self):
        self.port.write(self.AT_COMMAND["write_eeprom"].encode())
        while True:
            line = self.port.readline().decode("utf-8")
            if "OK" in line:
                break
            if line == "":
                print("Failed to write eeprom.")
                return False

    def _write_param(self, register, value):
        data = "ATS{}={}\r\n".format(register, value).encode()
        self.port.write(data)

        while True:
            line = self.port.readline().decode("utf-8")
            if "OK" in line:
                break
            if line == "":
                return False
        return True

    def write_param(self, name, value):
        if name not in self.EEPROM_PARAMETERS:
            return False
        if self.EEPROM_PARAMETERS[name]["readonly"]:
            return True
        register = self.EEPROM_PARAMETERS[name]["register"]
        return self._write_param(register, value)

    def check_param(self, name, value):
        if name not in self.EEPROM_PARAMETERS:
            return False
        if self.EEPROM_PARAMETERS[name]["options"] is None:
            return True
        else:
            return int(value) in self.EEPROM_PARAMETERS[name]["options"]

    def _parse_param(self):
        value = None
        while True:
            answer = self.port.readline().decode("utf-8")
            if not answer:
                break
            try:
                value = int(answer.split()[-1])
            except ValueError:
                pass
            else:
                break
        return value

    def read_param(self, name):
        if name not in self.EEPROM_PARAMETERS:
            return None
        register = self.EEPROM_PARAMETERS[name]["register"]
        self.port.reset_input_buffer()
        self.port.write("ATS{}?\r\n".format(register).encode())
        self.port.flush()
        return self._parse_param()

    def enter_at_mode(self, retries=3):
        success = False
        while True:
            print("Entering AT mode...")
            for i in range(2):
                self.exit_at_mode()
            time.sleep(1.05)
            self.port.write(b"+++")
            self.port.flush()
            self.port.reset_input_buffer()
            time.sleep(1.05)
            success = self.read_until_ok()
            if success:
                self.port.flushInput()
                return success
            print("Could not enter AT mode. Keep trying...")
            retries -= 1
            print("{} retries left".format(retries))
            if retries == 0:
                self.port.flushInput()
                return False

    def exit_at_mode(self):
        try:
            self.port.write(self.AT_COMMAND["exit"].encode())
        except serial.serialutil.SerialException:
            return False
        return True

    def reboot(self):
        try:
            self.port.write(self.AT_COMMAND["reboot"].encode())
        except serial.serialutil.SerialException:
            return False
        return True

    def show_version(self):
        self.port.reset_input_buffer()
        self.port.write(self.AT_COMMAND["show_version"].encode())
        version = ""
        while True:
            bytes = self.port.readline()
            print(bytes)
            answer = bytes.decode("utf-8")
            if not answer:
                break
            if answer == self.AT_COMMAND["show_version"]:
                continue
            version = answer
            break
        return version

    def read_parameters_from_eeprom(self):
        pattern = self._get_param_pattern()
        if pattern is None:
            return None

        parameters = {}
        self.port.write(self.AT_COMMAND["show_eeprom"].encode())
        while True:
            answer = self.port.readline().decode("utf-8")
            answer = answer.lstrip().rstrip()
            answer = answer.replace(" ", "")
            if not answer:
                break
            match = pattern.search(answer)
            if match is not None:
                parameters[match.group(1).lower()] = int(match.group(2))
        self._current_params = parameters
        return parameters

    def _read_param_format(self):
        self.port.reset_input_buffer()
        self.port.write("ATS0?\r\n".encode())
        self.port.flush()
        time.sleep(1.0)
        param_format = None
        while True:
            answer = self.port.readline().decode("utf-8")
            if not answer:
                break
            try:
                param_format = int(answer.split()[-1])
            except ValueError:
                pass
            else:
                break
        return param_format

    def _get_param_pattern(self):
        param_format = self._read_param_format()
        pattern = None
        if param_format == 26 or param_format == 27:
            pattern = re.compile(r"[0-9]+:([a-zA-z_]*)=([0-9]+)")
        else:
            print("Paramter format '{}' not supported. Please implement it!".
                  format(param_format))
        return pattern


def main():
    fw = Firmware("../radio~hm_trp.ihx")
    for address in sorted(fw.code().keys()):
        print(address, fw.code()[address])
        print()


if __name__ == "__main__":
    main()
