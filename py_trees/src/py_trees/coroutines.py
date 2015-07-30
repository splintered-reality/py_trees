#!/usr/bin/env python

import string
import inspect


class Status(object):
    """ An enumerator representing the status of an act """
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class Behaviour(object):
    """ A node in a behavior tree that uses coroutines in its tick function """

    def __init__(self, children=[], name="", *args, **kwargs):
        self.name = name
        self.iterator = self.tick()
        if children is None:
            children = []

        self.children = children

    def tick(self):
        pass

    def reset(self):
        self.iterator = self.tick()
        for child in self.children:
            child.reset()

    def add_child(self, child):
        self.children.append(child)

    def remove_child(self, child):
        self.children.remove(child)

    def suspend(self):
        pass

    def resume(self):
        pass

    def __enter__(self):
        return self.name

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            return False
        return True


class Selector(Behaviour):
    """
    Runs all of its child acts in sequence until one succeeds.
    """

    def __init__(self, children=[], name="Selector", *args, **kwargs):
        super(Selector, self).__init__(children, name, *args, **kwargs)
        self.current_child = None

    def tick(self):
        for child in self.children:
            self.current_child = child
            for status in child.iterator:
                if status == Status.RUNNING:
                    yield status
                elif status == Status.FAILURE:
                    yield Status.RUNNING
                else:
                    yield status
                    return
        yield Status.FAILURE

    def suspend(self):
        if self.current_child:
            self.current_child.suspend()

    def resume(self):
        if self.current_child:
            self.current_child.resume()

    def reset(self):
        self.current_child = None
        Behaviour.reset(self)


class Sequence(Behaviour):
    """
    Runs all of its children in sequence until all succeed
    """

    def __init__(self, children=[], name="Sequence", *args, **kwargs):
        super(Sequence, self).__init__(children, name, *args, **kwargs)
        self.current_child = None

    def tick(self):
        for child in self.children:
            current_child = child
            for status in child.iterator:
                if status == Status.RUNNING:
                    yield status
                elif status == Status.FAILURE:
                    yield Status.FAILURE
                    return
                else:
                    yield Status.RUNNING

        yield Status.SUCCESS

    def suspend(self):
        if self.current_child:
            self.current_child.suspend()

    def resume(self):
        if self.current_child:
            self.current_child.resume()

    def reset(self):
        self.current_child = None
        Behaviour.reset(self)


class Parallel(Behaviour):
    """
    Runs all of its children in parallel until one succeeds
    """

    def __init__(self, children=[], name="Parallel", *args, **kwargs):
        super(Parallel, self).__init__(children, name, *args, **kwargs)

    def tick(self):
        num_fails = 0
        while True:
            if num_fails >= len(self.children):
                yield Status.FAILURE
                return
            try:
                for child in self.children:
                    status = child.iterator.next()

                    if status == Status.SUCCESS:
                        yield Status.SUCCESS
                        return
                    elif status == Status.FAILURE:
                        num_fails += 1

            except StopIteration:
                continue
        yield Status.FAILURE

    def suspend(self):
        for child in self.children:
            child.suspend()

    def resume(self):
        for child in self.children:
            child.resume()


class Loop(Behaviour):
    """
    Runs all of its children for some number of iters, regardless of whether one succeeds. If iterations is set to
    -1, loops indefinitely. If any child fails, returns FAILURE.
    """

    def __init__(self, children = [], name = "Loop", num_iter = -1, *args, **kwargs):
        super(Loop, self).__init__(children, name, *args, **kwargs)
        self.num_iter = num_iter
        self.iter = 1
        self.current_child = None

    def tick(self):
        while True:
            if self.num_iter != -1 and self.iter >= self.num_iter:
                yield Status.SUCCESS
                return

            for child in self.children:
                current_child = child
                for status in child.iterator:
                    if status == Status.RUNNING:
                        yield status
                    elif status == Status.SUCCESS:
                        yield Status.RUNNING
                    elif status == Status.FAILURE:
                        yield Status.FAILURE
                        return
                child.reset()
            self.iter += 1
        yield Status.SUCCESS

    def suspend(self):
        if self.current_child:
            self.current_child.suspend()

    def resume(self):
        if self.current_child:
            self.current_child.resume()


class IgnoreFail(Behaviour):
    """
        Always return either RUNNING or SUCCESS.
    """

    def __init__(self, children=[], name="IgnoreFail", *args, **kwargs):
        super(IgnoreFail, self).__init__(children, name, *args, **kwargs)
        self.current_child = None

    def tick(self):
        for child in self.children:
            for status in child.iterator:
                if status == Status.FAILURE:
                    yield Status.SUCCESS
                else:
                    yield status
        yield Status.SUCCESS

    def suspend(self):
        if self.current_child:
            self.current_child.suspend()

    def resume(self):
        if self.current_child:
            self.current_child.resume()


class Not(Behaviour):
    """
        Returns FAILURE on SUCCESS, and SUCCESS on FAILURE
    """

    def __init__(self, children=[], name="Not", *args, **kwargs):
        super(Not, self).__init__(children, name, *args, **kwargs)
        self.current_child = None

    def tick(self):
        for child in self.children:
            current_child = child
            for status in child.iterator:
                if status == Status.FAILURE:
                    yield Status.SUCCESS
                elif status == Status.SUCCESS:
                    yield Status.FAILURE
                else:
                    yield status
        yield Status.SUCCESS

    def reset(self):
        self.current_child = None
        Behaviour.reset(self)

    def suspend(self):
        if self.current_child:
            self.current_child.suspend()

    def resume(self):
        if self.current_child:
            self.current_child.resume()


class Wrap(Behaviour):
    """
    Wraps a function that returns FAILURE or SUCCESS
    """

    def __init__(self, wrap_function, children=[], name="", *args, **kwargs):
        super(Wrap, self).__init__(children, name, *args, **kwargs)
        self.fn = wrap_function
        self.name = "Wrap " + inspect.getsource(self.fn)

    def tick(self):
        yield self.fn()

##############################################################################
# Display
##############################################################################


def print_ascii_tree(printTree, indent=0):
    """
    Print the ASCII representation of an act tree.
    :param tree: The root of an act tree
    :param indent: the number of characters to indent the tree
    :return: nothing
    """
    for child in printTree.children:
        print "    " * indent, "-->", child.name

        if child.children != []:
            print_ascii_tree(child, indent + 1)


import pygraphviz as pgv
from pygraph.classes.digraph import digraph
from pygraph.algorithms.searching import breadth_first_search
from pygraph.readwrite.dot import write
import gv


def print_dot_tree(root):
    """
        Print an output compatible with the DOT synatax and Graphiz
    """
    gr = pgv.AGraph(rotate='0', bgcolor='lightyellow')
    gr.node_attr['fontsize'] = '9'

    def add_edges(root):
        for c in root.children:
            gr.add_edge((root.name, c.name))
            if c.children != []:
                add_edges(c)

    add_edges(root)

    st, unused_order = breadth_first_search(gr, root=root.name)

    gst = digraph()
    gst.add_spanning_tree(st)

    dot = write(gst)
    gvv = gv.readstring(dot)

    gv.layout(gvv, 'dot')
    gv.render(gvv, 'png', 'tree.png')
    gv.render(gvv, 'svg', 'tree.svg')
    gv.write(gvv, 'tree.dot')

##############################################################################
# Mock Program
##############################################################################


def printobj(text):
    print text
    return Status.SUCCESS

if __name__ == "__main__":
    tree = Parallel(
        [
            Loop(
                [
                    Wrap(lambda: printobj("Hello 1")),
                    Wrap(lambda: printobj("Hello 2"))
                ], num_iter=10),
            Loop(
                [
                    Wrap(lambda: printobj("Hello 3")),
                    Wrap(lambda: printobj("Hello 4"))
                ], num_iter=5),
        ]
    )
    print_ascii_tree(tree)

    tree.reset()

    for status in tree.iterator:
        pass
