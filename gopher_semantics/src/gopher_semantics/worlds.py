#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: worlds
   :platform: Unix
   :synopsis: Semantics for worlds.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import gopher_semantic_msgs.msg as gopher_semantic_msgs
import rocon_console.console as console
import rospy

##############################################################################
# Worlds
##############################################################################


class Worlds(dict):
    '''
    List of the known worlds in the universe. Should be provided in yaml
    in the following form:

    .. code-block:: yaml

       worlds:
         - heaven
         - earth
         - hell

    Parameters:

    - ~semantics_parameter_namespace : where it can find semantics information on the ros parameter server.

    .. warning:: *default* is a reserved keyword, do not use it for a world's unique name.
    '''
    def __init__(self, semantics_parameter_namespace=None):
        super(Worlds, self).__init__()
        if semantics_parameter_namespace is None:
            semantics_parameter_namespace = rospy.get_param('~semantics_parameter_namespace', rospy.resolve_name('~'))
        parameters = rospy.get_param(semantics_parameter_namespace + "/worlds", {})
        self.default = None
        for unique_name, fields in parameters.iteritems():
            try:
                world = gopher_semantic_msgs.World()
                world.unique_name = unique_name
                world.name = fields['name']
                world.description = fields['description']
                self.__setitem__(unique_name, world)
            except KeyError:
                rospy.logwarn("Worlds : one of the expected fields for elevator '%s' was missing!" % unique_name)
            if 'default' in fields.keys():
                if fields['default']:
                    if self.default is not None:
                        rospy.logwarn("Semantics: you have multiple worlds flagged as the default, overriding [%s->%s]" % (self.default, unique_name))
                    self.default = unique_name

    def __str__(self):
        s = console.bold + "\nWorlds:\n" + console.reset
        for name in sorted(self):
            s += console.green + "  %s\n" % name
            world = dict.__getitem__(self, name)
            for key in world.__slots__:
                value = getattr(world, key)
                s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (value if value is not None else '-')
        s += console.reset
        return s

    def to_msg(self):
        msg = gopher_semantic_msgs.Worlds()
        msg.worlds = self.values()
        return msg

    def spin(self):
        rospy.spin()
