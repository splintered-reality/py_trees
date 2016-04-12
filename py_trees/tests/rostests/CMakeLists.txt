##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)
add_rostest(subscriber_check/subscriber_check.test)
add_rostest(subscriber_check/subscriber_nested_check.test)
