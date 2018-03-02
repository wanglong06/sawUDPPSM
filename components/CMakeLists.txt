#
# $Id: CMakeLists.txt 4588 2013-12-04 22:53:51Z adeguet1 $
#
# (C) Copyright 2014 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---
project(sawUDPPSM)
cmake_minimum_REQUIRED(VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawUDPPSM_IS_CATKIN_BUILT)
  if (sawUDPPSM_IS_CATKIN_BUILT)
    set (sawUDPPSM_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/share/sawUDPPSM/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawUDPPSM_BINARY_DIR}/lib")
  else ()
    set (sawUDPPSM_CONFIG_FILE_DIR "${sawUDPPSM_BINARY_DIR}")
  endif ()


    # create/configure file for find_package (sawUDPPSM)
    set (sawUDPPSM_INCLUDE_DIR "${sawUDPPSM_SOURCE_DIR}/include"
                               "${sawUDPPSM_BINARY_DIR}/include")
    set (sawUDPPSM_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")    
    set (sawUDPPSM_LIBRARIES sawUDPPSM)
    

    include_directories (${sawUDPPSM_INCLUDE_DIR})
    set (sawUDPPSM_HEADER_DIR "${sawUDPPSM_SOURCE_DIR}/include/sawUDPPSM")

    set (HEADER_FILES
         ${sawUDPPSM_HEADER_DIR}/mtsUDPPSM.h
         ${sawUDPPSM_HEADER_DIR}/mtsUDPMaster.h
        )

    set (SOURCE_FILES
         code/mtsUDPPSM.cpp
         code/mtsUDPMaster.cpp
        )

    add_library (sawUDPPSM ${HEADER_FILES} ${SOURCE_FILES})
    cisst_target_link_libraries (sawUDPPSM ${REQUIRED_CISST_LIBRARIES})
    set_property (TARGET sawUDPPSM PROPERTY FOLDER "sawUDPPSM")

    set (sawUDPPSM_CMAKE_CONFIG_FILE
         "${sawUDPPSM_CONFIG_FILE_DIR}/sawUDPPSMConfig.cmake")

    configure_file (
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawUDPPSMConfig.cmake.in"
      "${sawUDPPSM_CMAKE_CONFIG_FILE}")

    # Install targets for headers and library
    install (DIRECTORY
             ${sawUDPPSM_SOURCE_DIR}/include/sawUDPPSM
             ${sawUDPPSM_BINARY_DIR}/include/sawUDPPSM
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawUDPPSM
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawUDPPSM_CMAKE_CONFIG_FILE}"
             DESTINATION "share/sawUDPPSM")



endif (cisst_FOUND)
