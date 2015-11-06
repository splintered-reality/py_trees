# Possible behaviour states
uint8 INVALID = 1
uint8 RUNNING = 2
uint8 SUCCESS = 3
uint8 FAILURE = 4

# Possible types of behaviour
uint8 BEHAVIOUR = 1
uint8 SEQUENCE = 2
uint8 SELECTOR = 3

# Name given to the behaviour on construction
string name

# The ID of this behaviour
uuid_msgs/UniqueID own_id

# The ID of the parent behaviour (what if it's empty?)
uuid_msgs/UniqueID parent_id

# The IDs of children of this behaviour, if any
uuid_msgs/UniqueID[] child_ids

# A composite, sequence, selector, plain behaviour, etc.
uint8 type

# The current state of execution. Can be invalid, running, success or failure
uint8 status

# The message attached to the current execution state
string message