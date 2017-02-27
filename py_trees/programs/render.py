#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees.programs.render
   :func: command_line_argument_parser
   :prog: py-trees-render

.. image:: images/render.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import importlib
import json
import py_trees
import sys

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def examples():
    examples = [console.cyan + "py-trees-render" + console.yellow + " py_trees.demos.stewardship.create_tree" + console.reset,
                console.cyan + "py-trees-render" + console.yellow + " --name=foo py_trees.demos.stewardship.create_tree" + console.reset,
                console.cyan + "py-trees-render" + console.yellow + " --kwargs='{\"level\":\"all\"}' py_trees.demos.dot_graphs.create_tree" + console.reset
                ]
    return examples


def description():
    short = "Point this program at a method which creates a root to render to dot/svg/png.\n\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Trees".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += short
        s += "\n"
        s += console.bold + "Examples" + console.reset + "\n\n"
        s += '\n'.join(["    $ " + example for example in examples()])
        s += "\n\n"
        s += banner_line
    else:
        # for sphinx documentation (doesn't like raw text)
        s = short
        s += "\n"
        s += console.bold + "**Examples**" + console.reset + "\n\n"
        s += ".. code-block:: bash\n"
        s += "    \n"
        s += '\n'.join(["    $ {0}".format(example) for example in examples()])
        s += "\n"
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument('method', default=None, help='space separated list of blackboard variables to watch')
    parser.add_argument('-l', '--level', action='store',
                        default='fine_detail',
                        choices=['all', 'fine_detail', 'detail', 'component', 'big_picture'],
                        help='visibility level')
    parser.add_argument('-n', '--name', default=None, help='name to use for the created files (defaults to the root behaviour name)')
    parser.add_argument('-k', '--kwargs', default="{}", type=json.loads, help='dictionary of keyword arguments to the method')
    return parser


##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point.
    """
    args = command_line_argument_parser().parse_args()
    args.enum_level = py_trees.common.string_to_visibility_level(args.level)
    (module_name, method_name) = args.method.rsplit(".", 1)

    try:
        module_itself = importlib.import_module(module_name)
    except ImportError:
        console.logerror("Could not import module [{0}]".format(module_name))
        sys.exit(1)
    method_itself = getattr(module_itself, method_name)

    # TODO figure out how to insert keyword arguments
    root = method_itself(**(args.kwargs))
    py_trees.display.render_dot_tree(root, args.enum_level, args.name)
