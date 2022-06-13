#! /usr/bin/env python3

import time
from font import Font


class Logger(object):
    @staticmethod
    def LogDebug(msg):
        ts_ns = time.time() * 1e6
        print(f'{Font.BLUE}[{ts_ns}] {msg} {Font.END}')

    @staticmethod
    def LogInfo(msg):
        ts_ns = time.time() * 1e6
        print(f'[{ts_ns}] {msg}')

    @staticmethod
    def LogWarn(msg):
        ts_ns = time.time() * 1e6
        print(f'{Font.YELLOW}[{ts_ns}] {msg} {Font.END}')

    @staticmethod
    def LogERROR(msg):
        ts_ns = time.time() * 1e6
        print(f'{Font.RED}[{ts_ns}] {msg} {Font.END}')

    @staticmethod
    def LogFATAL(msg):
        ts_ns = time.time() * 1e6
        print(f'{Font.RED}{FONT.BOLD} [{ts_ns}] {msg} {Font.END}')
        exit()


if __name__ == '__main__':
    Logger.LogDebug("This is a debug message.")
    Logger.LogInfo("This is a info message.")
    Logger.LogWarn("This is a warning message.")
    Logger.LogERROR("This is a error message.")
