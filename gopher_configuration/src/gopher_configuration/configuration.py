#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: configuration
   :platform: Unix
   :synopsis: Gopher configuration in a usable structure.

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
import rocon_python_comms
import rospkg
import rospy
import socket
import yaml

##############################################################################
# Configuration
##############################################################################


class ConfigurationGroup(object):
    def __str__(self):
        s = console.green + "  " + type(self).__name__ + "\n" + console.reset
        for key, value in self.__dict__.iteritems():
            s += console.cyan + "    " + '{0: <20}'.format(key) + console.reset + ": " + console.yellow + "%s\n" % (value if value is not None else '-')
        s += console.reset
        return s


class LEDPatterns(ConfigurationGroup):
    """
    Self-defined group that associates certain colours with various actions.

    Unfortunately can't put these in the yaml yet. To do that, need to have string mappings to these
    message values. Maybe they themselves can be strings?
    """
    def __init__(self):
        # human interactions
        self.humans_give_me_input = gopher_std_msgs.Notification.FLASH_BLUE
        self.humans_be_careful = gopher_std_msgs.Notification.FLASH_YELLOW
        self.humans_i_need_help = gopher_std_msgs.Notification.FLASH_PURPLE
        self.humans_fix_me_i_am_broken = gopher_std_msgs.Notification.FLASH_RED
        # notifications
        self.im_doing_something_cool = gopher_std_msgs.Notification.AROUND_RIGHT_BLUE
        self.holding = gopher_std_msgs.Notification.AROUND_RIGHT_GREEN
        self.dab_dab_hae = gopher_std_msgs.Notification.AROUND_RIGHT_YELLOW
        # states
        self.error = gopher_std_msgs.Notification.SOLID_RED

    def validate(self):
        return (True, None)

    def __str__(self):
        s = console.green + "  " + type(self).__name__ + "\n" + console.reset
        s += console.cyan + "    " + '{0: <20}'.format("humans_give_me_input") + console.reset + ": " + console.yellow + "flashing blue\n"
        s += console.cyan + "    " + '{0: <20}'.format("humans_be_careful") + console.reset + ": " + console.yellow + "flashing yellow\n"
        s += console.cyan + "    " + '{0: <20}'.format("humans_i_need_help") + console.reset + ": " + console.yellow + "flashing purple\n"
        s += console.cyan + "    " + '{0: <20}'.format("humans_fix_me_i_am_broken") + console.reset + ": " + console.yellow + "flashing red\n"
        s += console.cyan + "    " + '{0: <20}'.format("holding") + console.reset + ": " + console.yellow + "around blue\n"
        s += console.cyan + "    " + '{0: <20}'.format("error") + console.reset + ": " + console.yellow + "solid red\n"
        return s


class Battery(ConfigurationGroup):
    def __init__(self, battery_dict):
        """
        :raises KeyError: if the provided dictionary doesn't contain the required keys.
        """
        # The set of battery variables is fixed, no __dict__ magic necessary.
        self.low = battery_dict['low']
        self.mid = battery_dict['mid']

    def validate(self):
        for key in ['low', 'mid']:
            if key not in self.__dict__.keys():
                return (False, "no definition for battery field '%s'" % key)
        return (True, None)


class Buttons(ConfigurationGroup):
    def __init__(self, buttons_dict):
        self.__dict__ = buttons_dict

    def validate(self):
        for key in ['go', 'stop']:
            if key not in self.__dict__.keys():
                return (False, "no definition for button '%s'" % key)
        return (True, None)


class Frames(ConfigurationGroup):
    def __init__(self, frames_dict):
        self.__dict__ = frames_dict

    def validate(self):
        for key in ['map']:
            if key not in self.__dict__.keys():
                return (False, "no definition for frame '%s'" % key)
        return (True, None)


class Actions(ConfigurationGroup):
    def __init__(self, actions_dict):
        self.__dict__ = actions_dict

    def validate(self):
        for key in ['teleport']:
            if key not in self.__dict__.keys():
                return (False, "no definition for action '%s'" % key)
        return (True, None)


class Services(ConfigurationGroup):
    def __init__(self, services_dict):
        self.__dict__ = services_dict

    def validate(self):
        for key in ['clear_costmaps']:
            if key not in self.__dict__.keys():
                return (False, "no definition for service '%s'" % key)
        return (True, None)


class Sounds(ConfigurationGroup):
    def __init__(self, sounds_dict):
        self.__dict__ = sounds_dict

    def validate(self):
        for key in ['honk']:
            if key not in self.__dict__.keys():
                return (False, "no definition for sound '%s'" % key)
        return (True, None)


class Topics(ConfigurationGroup):
    def __init__(self, topics_dict):
        self.__dict__ = topics_dict

    def validate(self):
        for key in ['display_notification', 'initial_pose', 'switch_map']:
            if key not in self.__dict__.keys():
                return (False, "no definition for topic '%s'" % key)
        return (True, None)


class Namespaces(ConfigurationGroup):
    def __init__(self, namespaces_dict):
        self.__dict__ = namespaces_dict

    def validate(self):
        for key in ['semantics']:
            if key not in self.__dict__.keys():
                return (False, "no definition for namespace '%s'" % key)
        return (True, None)


class Configuration(object):
    """
    Shared parameter storage/instantiation for gopher behaviours.
    This uses the `Borg Pattern`_, so feel free to instantiate as many times
    inside a process as you wish.


    **Launching a Configuration on the RosParam Server**:

    The simplest use case is to load your configuration and its customisation on the
    rosparam server in the ``/gopher/configuration`` namespace. This namespace is
    the default lookup location for this class.

    To do this, include a snippet like that below to load it for your robot:

    .. code-block:: xml
           :linenos:

           <launch>
               <rosparam ns="/gopher/configuration" command="load" file="$(find gopher_configuration)/param/defaults.yaml"/>
               <rosparam ns="/gopher/configuration" command="load" file="$(find foo_configuration)/param/customisation.yaml"/>
           <launch>


    Instantiate this class as an interface to the rosparam configuration. This class will fallback to
    loading defaults if you use the ``fallback_to_defaults=True``, useful for simulations.

    .. code-block:: python
       :linenos:

       try:
           gopher = gopher_configuration.Configuration()
           print("%s" % gopher)
           print("%s" % gopher.topics)
           print("Honk topic name: %s" % gopher.sounds.honk)
           print("Global frame id: %s" % gopher.frames.global)
        except ValueError as e:
           print("Configuration parameters found, but %s" % str(e))
        except rocon_python_comms.exceptions.ROSNotFound:
           print("No ros, if you want to fallback to showing defaults, pass in fallback_to_defaults=True or use Configuration.load_from_yaml")
        except rocon_python_comms.exceptions.NotFoundException as e:
           print("No configuration found on the rosparam server" % str(e))

    .. _Borg Pattern: http://code.activestate.com/recipes/66531-singleton-we-dont-need-no-stinkin-singleton-the-bo/

    """
    # two underscores for class private variable
    #   http://stackoverflow.com/questions/1301346/the-meaning-of-a-single-and-a-double-underscore-before-an-object-name-in-python
    __shared_state = {}

    @staticmethod
    def load_defaults():
        """
        Loads the default yaml, useful for referencing the default configuration.
        This is used by the ``gopher_configuration`` command line script.
        """
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("gopher_configuration")
        filename = os.path.join(pkg_path, "param", "defaults.yaml")
        Configuration.__shared_state = yaml.load(open(filename))

    @staticmethod
    def load_from_rosparam_server(namespace='/gopher/configuration'):
        """
        This automatically gets called if you try to instantiate before
        it has retrieved anything from the rosparam server.
        It can also be called by the user to point the configuration
        to a different location on the rosparam server for all
        future instances of this class. This is not a common use
        case though.

        As an example, to load and retrieve configuration from a
        namespace called ``foo``:

        .. code-block:: xml
           :linenos:

           <launch>
               <rosparam ns="/foo/configuration" command="load" file="$(find gopher_configuration)/param/defaults.yaml"/>
               <rosparam ns="/foo/configuration" command="load" file="$(find foo_configuration)/param/customisation.yaml"/>
           <launch>

        .. code-block:: python
           :linenos:

           gopher_configuration.Configuration.load_from_rosparam_server(namespace='/foo/configuration')
           gopher = gopher_configuration.Configuration()

           :raises rocon_python_comms.ROSNotFoundException: if ros is not around
           :raises rocon_python_comms.NotFoundException: configuration parameters completely not found on the rosparam server.
           :raises ValueError: if missing or improperly set at least one of the required 'core' parameters.
        """
        try:
            Configuration.__shared_state = rospy.get_param(namespace)
        except KeyError:
            raise KeyError(namespace)

    def __init__(self, fallback_to_defaults=False):
        """
        :param fallback_to_defaults: if not parameterised already and rosparam lookup fails, load gopher defaults from yaml.
        """
        if not Configuration.__shared_state:
            try:
                Configuration.load_from_rosparam_server()
            except KeyError as e:
                if fallback_to_defaults:
                    Configuration.load_defaults()
                else:
                    # ros is around, but nothing on the parameter server
                    raise(rocon_python_comms.exceptions.NotFoundException(str(e)))
            except socket.error:
                if fallback_to_defaults:
                    Configuration.load_defaults()
                else:
                    raise rocon_python_comms.exceptions.ROSNotFoundException("no ros found")
        core = ['actions', 'battery', 'buttons', 'namespaces', 'frames', 'topics', 'services', 'sounds', 'led_patterns']
        try:
            # catch our special groups
            self.actions      = Actions(Configuration.__shared_state['actions'])        # @IgnorePep8
            self.battery      = Battery(Configuration.__shared_state['battery'])        # @IgnorePep8
            self.buttons      = Buttons(Configuration.__shared_state['buttons'])        # @IgnorePep8
            self.namespaces   = Namespaces(Configuration.__shared_state['namespaces'])  # @IgnorePep8
            self.frames       = Frames(Configuration.__shared_state['frames'])          # @IgnorePep8
            self.sounds       = Sounds(Configuration.__shared_state['sounds'])          # @IgnorePep8
            self.services     = Services(Configuration.__shared_state['services'])      # @IgnorePep8
            self.topics       = Topics(Configuration.__shared_state['topics'])          # @IgnorePep8
            self.led_patterns = LEDPatterns()
        except KeyError:
            raise ValueError("at least one of the core parameter groups missing %s" % core)
        for name in core:
            parameter_group = getattr(self, name)
            (result, error_message) = parameter_group.validate()
            if not result:
                raise ValueError("%s" % error_message)
        # catch everything else
        for key, value in Configuration.__shared_state.iteritems():
            if key not in core:
                setattr(self, key, value)

    def __str__(self):
        if not self.__dict__:
            return ""
        s = console.bold + "\nGopher Configuration:\n\n" + console.reset
        for key, value in self.__dict__.iteritems():
            if isinstance(value, ConfigurationGroup):
                s += ("%s" % value)
            else:
                s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (value if value is not None else '-')
        s += console.reset
        return s
