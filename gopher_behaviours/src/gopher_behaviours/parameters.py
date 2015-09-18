#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: parameters
   :platform: Unix
   :synopsis: Gopher deliveries configuration in a usable structure.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import gopher_std_msgs.msg as gopher_std_msgs
import os
import rocon_console.console as console
import rospkg
import rospy
import yaml

##############################################################################
# Parameters
##############################################################################


def _error_logger(msg):
    print(console.red + "Parameters : %s" % msg + console.reset)

##############################################################################
# Parameters
##############################################################################


class ParameterGroup(object):
    def __str__(self):
        s = console.green + "  " + type(self).__name__ + "\n" + console.reset
        for key, value in self.__dict__.iteritems():
            s += console.cyan + "    " + '{0: <20}'.format(key) + console.reset + ": " + console.yellow + "%s\n" % (value if value is not None else '-')
        s += console.reset
        return s


class LEDPatterns(ParameterGroup):
    """
    Self-defined group that associates certain colours with various actions.
    """
    def __init__(self):
        # human interactions
        self.humans_give_me_input = gopher_std_msgs.Notification.FLASH_BLUE
        self.humans_be_careful = gopher_std_msgs.Notification.FLASH_YELLOW
        self.humans_i_need_help = gopher_std_msgs.Notification.FLASH_PURPLE
        # notifications
        self.holding = gopher_std_msgs.Notification.AROUND_RIGHT_BLUE
        # states
        self.error = gopher_std_msgs.Notification.SOLID_RED

    def validate(self):
        return (True, None)

    def __str__(self):
        s = console.green + "  " + type(self).__name__ + "\n" + console.reset
        s += console.cyan + "    " + '{0: <20}'.format("humans_give_me_input") + console.reset + ": " + console.yellow + "flashing blue\n"
        s += console.cyan + "    " + '{0: <20}'.format("humans_be_careful") + console.reset + ": " + console.yellow + "flashing yellow\n"
        s += console.cyan + "    " + '{0: <20}'.format("humans_i_need_help") + console.reset + ": " + console.yellow + "flashing purple\n"
        s += console.cyan + "    " + '{0: <20}'.format("holding") + console.reset + ": " + console.yellow + "around blue\n"
        s += console.cyan + "    " + '{0: <20}'.format("error") + console.reset + ": " + console.yellow + "solid red\n"
        return s


class Buttons(ParameterGroup):
    def __init__(self, buttons_dict):
        self.__dict__ = buttons_dict

    def validate(self):
        for key in ['go', 'stop']:
            if key not in self.__dict__.keys():
                return (False, "no definition for button '%s'" % key)
        return (True, None)


class Frames(ParameterGroup):
    def __init__(self, frames_dict):
        self.__dict__ = frames_dict

    def validate(self):
        for key in ['map']:
            if key not in self.__dict__.keys():
                return (False, "no definition for frame '%s'" % key)
        return (True, None)


class Services(ParameterGroup):
    def __init__(self, services_dict):
        self.__dict__ = services_dict

    def validate(self):
        for key in ['clear_costmaps']:
            if key not in self.__dict__.keys():
                return (False, "no definition for service '%s'" % key)
        return (True, None)


class Sounds(ParameterGroup):
    def __init__(self, sounds_dict):
        self.__dict__ = sounds_dict

    def validate(self):
        for key in ['honk']:
            if key not in self.__dict__.keys():
                return (False, "no definition for sound '%s'" % key)
        return (True, None)


class Topics(ParameterGroup):
    def __init__(self, topics_dict):
        self.__dict__ = topics_dict

    def validate(self):
        for key in ['display_notification', 'initial_pose', 'switch_map']:
            if key not in self.__dict__.keys():
                return (False, "no definition for topic '%s'" % key)
        return (True, None)


class Namespaces(ParameterGroup):
    def __init__(self, namespaces_dict):
        self.__dict__ = namespaces_dict

    def validate(self):
        for key in ['semantics']:
            if key not in self.__dict__.keys():
                return (False, "no definition for namespace '%s'" % key)
        return (True, None)


class Parameters(object):
    """
    Shared parameter storage/instantiation for gopher behaviours.

    This uses the Borg pattern.
        http://code.activestate.com/recipes/66531-singleton-we-dont-need-no-stinkin-singleton-the-bo/
    """

    # two underscores for class private variable
    #   http://stackoverflow.com/questions/1301346/the-meaning-of-a-single-and-a-double-underscore-before-an-object-name-in-python
    __shared_state = {}

    @staticmethod
    def load_defaults():
        """
        Loads the default yaml, useful for testing.
        """
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("gopher_behaviours")
        filename = os.path.join(pkg_path, "param", "defaults.yaml")
        Parameters.__shared_state = yaml.load(open(filename))

    @staticmethod
    def load_from_rosparam_server(namespace='~'):
        Parameters.__shared_state = rospy.get_param(namespace)

    def __init__(self, error_logger=_error_logger):
        if not Parameters.__shared_state:
            Parameters.load_from_rosparam_server()
        core = ['buttons', 'namespaces', 'frames', 'topics', 'services', 'sounds', 'led_patterns']
        try:
            # catch our special groups
            self.buttons      = Buttons(Parameters.__shared_state['buttons'])        # @IgnorePep8
            self.namespaces   = Namespaces(Parameters.__shared_state['namespaces'])  # @IgnorePep8
            self.frames       = Frames(Parameters.__shared_state['frames'])          # @IgnorePep8
            self.sounds       = Sounds(Parameters.__shared_state['sounds'])          # @IgnorePep8
            self.services     = Services(Parameters.__shared_state['services'])      # @IgnorePep8
            self.topics       = Topics(Parameters.__shared_state['topics'])          # @IgnorePep8
            self.led_patterns = LEDPatterns()
        except KeyError:
            error_logger("Gopher Deliveries : at least one of the core parameter groups missing %s" % core)
            return
        for name in core:
            parameter_group = getattr(self, name)
            (result, error_message) = parameter_group.validate()
            if not result:
                error_logger("Gopher Deliveries : %s" % error_message)
        # catch everything else
        for key, value in Parameters.__shared_state.iteritems():
            if key not in core:
                setattr(self, key, value)

    def __str__(self):
        s = console.bold + "\nParameters:\n" + console.reset
        for key, value in self.__dict__.iteritems():
            if isinstance(value, ParameterGroup):
                s += ("%s" % value)
            else:
                s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (value if value is not None else '-')
        s += console.reset
        return s
