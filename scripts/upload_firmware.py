#!/usr/bin/env python
import argparse
import serial
import time

from radio_tools import radio_tools

upload_progress = 0
verify_progress = 0
upload_started = False
verify_started = False
chars = ["\\", "|", "/", "|"]
MAX_PROGRESS = 50


def print_upload_progress(progress, total):
    global upload_started
    if not upload_started:
        print("\n\nUpload:")
        upload_started = True

    new_progress = int(MAX_PROGRESS * float(progress) / total)
    global upload_progress
    if new_progress > upload_progress:
        upload_progress = new_progress
        print("\r[{}{}]".format("#" * upload_progress,
                                " " * (MAX_PROGRESS - upload_progress)),
              end="")


def print_verify_progress(progress, total):
    global verify_started
    if not verify_started:
        print("\n\nVerify:")
        verify_started = True

    new_progress = int(MAX_PROGRESS * float(progress) / total)
    global verify_progress
    if new_progress > verify_progress:
        verify_progress = new_progress
        print("\r[{}{}]".format("#" * verify_progress,
                                " " * (MAX_PROGRESS - verify_progress)),
              end="")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", action="store", required=True)
    parser.add_argument("-b", "--baud", action="store", default=57600)
    parser.add_argument("firmware", action="store")
    args = parser.parse_args()
    uploader = radio_tools.HippoLinkUploader(args.port, args.baud)
    uploader.upload_firmware(args.firmware, print_upload_progress,
                             print_verify_progress)


def test():
    s = serial.Serial("/dev/ttyUSB0", baudrate=57600)
    time.sleep(1.0)
    s.reset_input_buffer()
    s.write(b"+++++")
    s.flush()
    while True:
        print(s.read_until(b"OK\r\n"))


if __name__ == "__main__":
    main()
