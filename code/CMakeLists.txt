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

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawRobotIO1394 REQUIRED)
  cisst_find_saw_component (sawControllers REQUIRED)
  cisst_find_saw_component (sawIntuitiveResearchKit REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND AND sawIntuitiveResearchKit_FOUND)

    # sawRobotIO1394 configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR}
                         ${sawIntuitiveResearchKit_INCLUDE_DIR})
                       
    link_directories (${sawRobotIO1394_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR}
                      ${sawIntuitiveResearchKit_LIBRARY_DIR})

    # create/configure file for find_package (sawUDPPSM)
    set (sawUDPPSM_INCLUDE_DIR "${sawUDPPSM_SOURCE_DIR}/include")
    set (sawUDPPSM_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawUDPPSM_LIBRARIES sawUDPPSM)

    include_directories (${sawUDPPSM_INCLUDE_DIR})
    set (sawUDPPSM_HEADER_DIR "${sawUDPPSM_INCLUDE_DIR}/sawUDPPSM")

    set (HEADER_FILES
         ${sawUDPPSM_HEADER_DIR}/mtsUDPPSM.h
        )

    set (SOURCE_FILES
         mtsUDPPSM.cpp
        )

    add_library (sawUDPPSM ${HEADER_FILES} ${SOURCE_FILES})
    cisst_target_link_libraries (sawUDPPSM ${REQUIRED_CISST_LIBRARIES})
    set_property (TARGET sawUDPPSM PROPERTY FOLDER "sawUDPPSM")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawUDPPSM
                           ${sawRobotIO1394_LIBRARIES}
                           ${sawControllers_LIBRARIES}
                           ${sawIntuitiveResearchKit_LIBRARIES}
                           )

    set (sawUDPPSM_LIBRARIES ${sawUDPPSM_LIBRARIES})

    configure_file (
      "${CMAKE_CURRENT_SOURCE_DIR}/sawUDPPSMConfig.cmake.in"
      "${sawUDPPSM_BINARY_DIR}/sawUDPPSMConfig.cmake"
      @ONLY@)

    # Install targets for headers and library
    install (DIRECTORY
             ${sawUDPPSM_INCLUDE_DIR}/sawUDPPSM
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawUDPPSM
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawUDPPSM_BINARY_DIR}/sawUDPPSMConfig.cmake"
             DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  endif (sawRobotIO1394_FOUND AND sawControllers_FOUND AND sawIntuitiveResearchKit_FOUND)

endif (cisst_FOUND)
