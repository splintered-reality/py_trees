##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

##############################################################################
# Nose Tests
##############################################################################

catkin_add_nosetests(nosetests)

##############################################################################
# Ros Tests
##############################################################################

#find_package(catkin REQUIRED COMPONENTS rostest)
#add_rostest(connection_cache/connection_cache.test)
#add_rostest(service_pairs/service_pairs.test)
#add_rostest(service_pair_timeouts/service_pair_timeouts.test)
#add_rostest(subscriber_proxy/subscriber_proxy.test)
#add_rostest(find_node/find_node.test)
