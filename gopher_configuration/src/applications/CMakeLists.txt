###############################################################################
# Macro
###############################################################################

macro(gopher_configuration_add_demo name)
  set(target_name ${PROJECT_NAME}_${name})
  add_executable(${target_name} ${name}.cpp)
  target_link_libraries(${target_name} ${catkin_LIBRARIES} ${PROJECT_NAME})
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME ${name})
  install(TARGETS ${target_name} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

###############################################################################
# Setup
###############################################################################

gopher_configuration_add_demo(demo_node)
