#! /usr/bin/env python3

import time
from .font import Font


class Logger(object):
    Verbosity = 1

    @staticmethod
    def LogDebug(msg):
        if Logger.Verbosity > 4:
            ts_ns = time.time() * 1e6
            print(f'{Font.BLUE}[{ts_ns}] {msg} {Font.END}')

    @staticmethod
    def LogInfo(msg):
        if Logger.Verbosity > 0:
            ts_ns = time.time() * 1e6
            print(f'[{ts_ns}] {msg}')

    @staticmethod
    def LogWarn(msg):
        if Logger.Verbosity > 1:
            ts_ns = time.time() * 1e6
            print(f'{Font.YELLOW}[{ts_ns}] {msg} {Font.END}')

    @staticmethod
    def LogError(msg):
        if Logger.Verbosity > 2:
            ts_ns = time.time() * 1e6
            print(f'{Font.RED}[{ts_ns}] {msg} {Font.END}')

    @staticmethod
    def LogFatal(msg):
        if Logger.Verbosity > 3:
            ts_ns = time.time() * 1e6
            print(f'{Font.RED}{FONT.BOLD} [{ts_ns}] {msg} {Font.END}')
            exit()


if __name__ == '__main__':
    Logger.LogDebug("This is a debug message.")
    Logger.LogInfo("This is a info message.")
    Logger.LogWarn("This is a warning message.")
    Logger.LogError("This is a error message.")
