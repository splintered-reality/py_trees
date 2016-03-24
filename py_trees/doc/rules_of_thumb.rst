Rules of Thumb
==============

Behaviours
----------

Stopping - Beware of Switching Branches
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From one run to the next, if the currently running part of the tree gets switched out for a higher
priority part, this will always happen via a ``Selector``. When this occurs, the selector *will* always
send a ``stop(INVALID)`` to the previously running part of the tree.

This ``stop(INVALID)`` call *must* percolate down through all previously running composites
and behaviours, so make sure your stop behaviours are well composed and thought out.

Behaviour Messages - What Should Go In Here?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*The Signficance of Change*

The most interesting/useful data from a behaviour tree are the decision events - i.e. exactly
*when* some decisional state in the behaviour tree changes. If you have a behaviour that is
running for many ticks - *leave the message fairly constant* unless there is a significant
event that is important to the user. This allows humans and higher level programs (e.g.
``rqt_py_trees`` to be more tuned into the important events without having to filter through
alot of noise to work it out.

A good example of this is the battery behaviour. You probably don't need to notify
the user of the percentage levels, just the full/high/low/emergency states so that
a higher level program can detect a transition and make a notification.
