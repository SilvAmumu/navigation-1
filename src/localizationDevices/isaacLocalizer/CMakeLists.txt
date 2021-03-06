#
# Copyright (C) 2019 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

yarp_prepare_plugin(isaacLocalizer
                    CATEGORY device
                    TYPE isaacLocalizer
                    INCLUDE isaacLocalizer.h
                    INTERNAL)
                                            
set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(isaacLocalizer isaacLocalizer.h isaacLocalizer.cpp)
                              
target_link_libraries(isaacLocalizer YARP::YARP_OS
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math)


yarp_install(TARGETS isaacLocalizer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES isaacLocalizer.ini
           COMPONENT ${YARP_PLUGIN_MASTER}
           DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET isaacLocalizer PROPERTY FOLDER "Plugins/Device")

