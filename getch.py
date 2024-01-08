#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Get a single char from console input in a blocking or non-blocking way."""

from __future__ import print_function

import sys
import select
import tty
import termios


def getch(timeout=None):
    """Return a single char from console input.

    The optional timeout argument specifies a time-out as a floating point
    number in seconds. When the timeout argument is omitted or None (the
    default) the function blocks until a char has been read. If the timeout is
    exceeded before a char can be read, the function returns None. A time-out
    value of zero specifies a poll and never blocks.

    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        rlist, _, _ = select.select([fd], [], [], timeout)
        if fd in rlist:
            return sys.stdin.read(1)

        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == '__main__':
    from time import sleep

    print("Entering loop...")
    while True:
        ch = getch(0.05)

        if ch and ch in '\n\r':
            print("Char:", repr(ch))
            break
