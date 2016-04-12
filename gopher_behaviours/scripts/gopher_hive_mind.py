#!/usr/bin/env python

import argparse
import gopher_behaviours
import py_trees
import rocon_console.console as console
import rospy
import sys


##############################################################################
# Arg Parsing
##############################################################################


def show_description():
    s = "\n"
    s += console.green + "This node runs the delivery behaviour tree for gophers." + console.reset
    s += "\n"
    return s


def show_usage():
    s = "\n"
    s += console.white
    s += console.bold + "************************************************************************************\n" + console.reset
    s += console.bold + "                                Gopher Hive Mind\n" + console.reset
    s += console.bold + "************************************************************************************\n" + console.reset
    s += "\n"
    s += console.white
    s += console.bold + "    Generate Dot" + console.reset + "\n"
    s += console.cyan + "        gopher_hive_mind.py" + console.yellow + " --render" + "[--visibility-level=all]" + console.reset + "\n"
    s += "\n"
    s += console.bold + "    RosLaunch" + console.reset + "\n"
    s += console.cyan + "        roslaunch gopher_behaviours gopher_deliveries.launch" + console.reset + "\n"
    s += "\n"
    s += console.white
    s += console.bold + "    Requrired Launchers" + console.reset + "\n"
    s += console.cyan + "        + gopher_robot/minimal.launch" + console.reset + "\n"
    s += console.cyan + "        + ...(many - bring up instead with rapps)" + console.reset + "\n"
    s += console.cyan + "        or" + console.reset
    s += console.cyan + "        + gopher_desktop/gocart_sim.launch" + console.reset + "\n"
    s += "\n"
    s += console.white
    s += console.bold + "    Rapps" + console.reset + "\n"
    s += console.cyan + "        + gopher_robot/gocart_delivery" + console.reset + "\n"
    s += console.cyan + "        + ...(many - bring up instead with rapps)" + console.reset + "\n"
    s += "\n"
    return s


def parse_arguments(command_line_args):
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(),
                                     epilog="And his noodly appendage reached forth to tickle the blessed...",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter
                                     )
    parser.add_argument('-r', '--render', action='store_true', help='render the graph to dot/png/svg.')
    parser.add_argument('-d', '--debug', action='store_true', help='debug level tree logging.')
    parser.add_argument('-v', '--visibility-level',
                        action='store',
                        default="detail",
                        choices=py_trees.common.visibility_level_strings,
                        help='visibility level for blackboxes when using -r'
                        )

    args = parser.parse_args(command_line_args)
    args.visibility_level = py_trees.common.string_to_visibility_level(args.visibility_level)
    return args

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    command_line_args = rospy.myargv(argv=sys.argv)[1:]
    args = parse_arguments(command_line_args)
    if args.render:
        gopher_compulsion = gopher_behaviours.hive_mind.GopherHiveMind()
        gopher_compulsion.render_dot_tree(args.visibility_level)
        sys.exit()

    # have to read debug param before initialising node, it's not possible to
    # change the log level after node is initialised.
    debug_mode = rospy.get_param("/gopher/deliveries/debug", False) or args.debug
    if debug_mode:
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("gopher_compulsion", log_level=rospy.INFO)

    gopher_compulsion = gopher_behaviours.hive_mind.GopherHiveMind()
    gopher_compulsion.setup(30)
    gopher_compulsion.tick_tock()
