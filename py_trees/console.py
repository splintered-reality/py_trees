#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Description
##############################################################################

"""
Simple colour definitions and syntax highlighting for the console.

----

**Colour Definitions**

The current list of colour definitions include:

 * ``Regular``: black, red, green, yellow, blue, magenta, cyan, white,
 * ``Bold``: bold, bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white

These colour definitions can be used in the following way:

.. code-block:: python

   import py_trees.console as console
   print(console.cyan + "    Name" + console.reset + ": " + console.yellow + "Dude" + console.reset)

"""

##############################################################################
# Imports
##############################################################################

import os
import sys


##############################################################################
# Special Characters
##############################################################################


def has_unicode(encoding: str=sys.stdout.encoding) -> bool:
    """
    Define whether the specified encoding has unicode symbols. Usually used to check
    if the stdout is capable or otherwise (e.g. Jenkins CI can often be configured
    with unicode disabled).

    Args:
        encoding (:obj:`str`, optional): the encoding to check against.

    Returns:
        :obj:`bool`: true if capable, false otherwise
    """
    try:
        u'\u26A1'.encode(encoding)
    except TypeError:
        # if sys.stdout.encoding is not available, it is None
        # this will occur if you run nosetests3 without -s
        return False
    except UnicodeError:
        return False
    return True


def define_symbol_or_fallback(original: str, fallback: str, encoding: str=sys.stdout.encoding):
    """
    Return the correct encoding according to the specified encoding. Used to
    make sure we get an appropriate symbol, even if the shell is merely ascii as
    is often the case on, e.g. Jenkins CI.

    Args:
        original (:obj:`str`): the unicode string (usually just a character)
        fallback (:obj:`str`): the fallback ascii string
        encoding (:obj:`str`, optional): the encoding to check against.

    Returns:
        :obj:`str`: either original or fallback depending on whether exceptions were thrown.
    """
    try:
        original.encode(encoding)
    except UnicodeError:
        return fallback
    return original


circle = u'\u26ac'
lightning_bolt = u'\u26A1'
double_vertical_line = u'\u2016'
check_mark = u'\u2713'
multiplication_x = u'\u2715'
left_arrow = u'\u2190'  # u'\u2190'
right_arrow = u'\u2192'
left_right_arrow = u'\u2194'
forbidden_circle = u'\u29B8'
circled_m = u'\u24c2'

##############################################################################
# Keypress
##############################################################################


def read_single_keypress():
    """Waits for a single keypress on stdin.

    This is a silly function to call if you need to do it a lot because it has
    to store stdin's current setup, setup stdin for reading single keystrokes
    then read the single keystroke then revert stdin back after reading the
    keystroke.

    Returns:
        :obj:`int`: the character of the key that was pressed

    Raises:
        KeyboardInterrupt: if CTRL-C was pressed (keycode 0x03)
    """
    def read_single_keypress_unix():
        """For Unix case, where fcntl, termios is available."""
        import fcntl
        import termios
        fd = sys.stdin.fileno()
        # save old state
        flags_save = fcntl.fcntl(fd, fcntl.F_GETFL)
        attrs_save = termios.tcgetattr(fd)
        # make raw - the way to do this comes from the termios(3) man page.
        attrs = list(attrs_save)  # copy the stored version to update
        # iflag
        attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                      termios.ISTRIP | termios.INLCR | termios. IGNCR |
                      termios.ICRNL | termios.IXON)
        # oflag
        attrs[1] &= ~termios.OPOST
        # cflag
        attrs[2] &= ~(termios.CSIZE | termios. PARENB)
        attrs[2] |= termios.CS8
        # lflag
        attrs[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON |
                      termios.ISIG | termios.IEXTEN)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        # turn off non-blocking
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)
        # read a single keystroke
        ret = sys.stdin.read(1)  # returns a single character
        if ord(ret) == 3:  # CTRL-C
            termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)
            raise KeyboardInterrupt("Ctrl-c")
        # restore old state
        termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)
        return ret

    def read_single_keypress_windows():
        """Windows case, can't use fcntl and termios.
        Not same implementation as for Unix, requires a newline to continue.
        """
        import msvcrt  # noqa
        # read a single keystroke
        ret = sys.stdin.read(1)
        if ord(ret) == 3:  # CTRL-C
            raise KeyboardInterrupt("Ctrl-c")
        return ret
    try:
        return read_single_keypress_unix()
    except ImportError as e_unix:
        try:
            return read_single_keypress_windows()
        except ImportError as e_windows:
            raise ImportError("Neither unix nor windows implementations supported [{}][{}]".format(str(e_unix), str(e_windows)))

##############################################################################
# Methods
##############################################################################


def console_has_colours():
    """
    Detects if the console (stdout) has colourising capability.
    """
    if os.environ.get("PY_TREES_DISABLE_COLORS"):
        return False
    # From django.core.management.color.supports_color
    #   https://github.com/django/django/blob/master/django/core/management/color.py
    plat = sys.platform
    supported_platform = plat != 'Pocket PC' and (plat != 'win32' or
                                                  'ANSICON' in os.environ)
    # isatty is not always implemented, #6223.
    is_a_tty = hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()
    if not supported_platform or not is_a_tty:
        return False
    return True


has_colours = console_has_colours()
""" Whether the loading program has access to colours or not."""


if has_colours:
    # reset = "\x1b[0;0m"
    reset = "\x1b[0m"
    bold = "\x1b[%sm" % '1'
    dim = "\x1b[%sm" % '2'
    underlined = "\x1b[%sm" % '4'
    blink = "\x1b[%sm" % '5'
    black, red, green, yellow, blue, magenta, cyan, white = ["\x1b[%sm" % str(i) for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["\x1b[%sm" % ('1;' + str(i)) for i in range(30, 38)]
else:
    reset = ""
    bold = ""
    dim = ""
    underlined = ""
    blink = ""
    black, red, green, yellow, blue, magenta, cyan, white = ["" for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["" for i in range(30, 38)]

colours = [bold, dim, underlined, blink,
           black, red, green, yellow, blue, magenta, cyan, white,
           bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white
           ]
"""List of all available colours."""


def pretty_print(msg, colour=white):
    if has_colours:
        seq = colour + msg + reset
        sys.stdout.write(seq)
    else:
        sys.stdout.write(msg)


def pretty_println(msg, colour=white):
    if has_colours:
        seq = colour + msg + reset
        sys.stdout.write(seq)
        sys.stdout.write("\n")
    else:
        sys.stdout.write(msg)


##############################################################################
# Console
##############################################################################


def banner(msg):
    print(green + "\n" + 80 * "*" + reset)
    print(green + "* " + bold_white + msg.center(80) + reset)
    print(green + 80 * "*" + "\n" + reset)


def debug(msg):
    print(green + msg + reset)


def warning(msg):
    print(yellow + msg + reset)


def info(msg):
    print(msg)


def error(msg):
    print(red + msg + reset)


def logdebug(message):
    '''
    Prefixes ``[DEBUG]`` and colours the message green.

    Args:
        message (:obj:`str`): message to log.
    '''
    print(green + "[DEBUG] " + message + reset)


def loginfo(message):
    '''
    Prefixes ``[ INFO]`` to the message.

    Args:
        message (:obj:`str`): message to log.
    '''
    print("[ INFO] " + message)


def logwarn(message):
    '''
    Prefixes ``[ WARN]`` and colours the message yellow.

    Args:
        message (:obj:`str`): message to log.
    '''
    print(yellow + "[ WARN] " + message + reset)


def logerror(message):
    '''
    Prefixes ``[ERROR]`` and colours the message red.

    Args:
        message (:obj:`str`): message to log.
    '''
    print(red + "[ERROR] " + message + reset)


def logfatal(message):
    '''
    Prefixes ``[FATAL]`` and colours the message bold red.

    Args:
        message (:obj:`str`): message to log.
    '''
    print(bold_red + "[FATAL] " + message + reset)


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    for colour in colours:
        pretty_print("dude\n", colour)
    logdebug("loginfo message")
    logwarn("logwarn message")
    logerror("logerror message")
    logfatal("logfatal message")
    pretty_print("red\n", red)
    print("some normal text")
    print(cyan + "    Name" + reset + ": " + yellow + "Dude" + reset)
    print("special characters are\n")
    print("lightning_bolt: {}".format(lightning_bolt))
    print("double_vertical_line: {}".format(double_vertical_line))
    print("check_mark: {}".format(check_mark))
    print("multiplication_x: {}".format(multiplication_x))
    print("left_arrow: {}".format(left_arrow))
    print("right_arrow: {}".format(right_arrow))
    print("circled_m: {}".format(circled_m))
    # print("has unicode: {}".format(has_unicode()))
