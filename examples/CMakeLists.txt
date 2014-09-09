#
# $Id: CMakeLists.txt 4639 2013-12-22 23:17:32Z adeguet1 $
#
# (C) Copyright 2009-2013 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
                              cisstVector
                              cisstVectorQt
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstMultiTaskQt
                              cisstParameterTypes
                              cisstParameterTypesQt
                              cisstRobot
                              cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawRobotIO1394 REQUIRED)
  cisst_find_saw_component (sawIntuitiveResearchKit REQUIRED)
  cisst_find_saw_component (sawUDPPSM REQUIRED)
  cisst_find_saw_component (sawControllers REQUIRED)
  cisst_find_saw_component (sawTextToSpeech REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND)

    # sawRobotIO1394 configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR}
                         ${sawIntuitiveResearchKit_INCLUDE_DIR}
                         ${sawUDPPSM_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR}
                         ${sawTextToSpeech_INCLUDE_DIR})
                       
    link_directories (${sawRobotIO1394_LIBRARY_DIR}
                      ${sawIntuitiveResearchKit_LIBRARY_DIR}
                      ${sawUDPPSM_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR}
                      ${sawTextToSpeech_LIBRARY_DIR})

    # examples using Qt
    if (CISST_HAS_QT)

      # example using JSON
      if (CISST_HAS_JSON)

        # main program used to configure TeleOperation
        add_executable (sawUDPPSMQtTeleOperationJSON mainQtTeleOperationJSON.cpp)
        set_property (TARGET sawUDPPSMQtTeleOperationJSON PROPERTY FOLDER "sawUDPPSM")

        # link against non cisst libraries and cisst components
        target_link_libraries (sawUDPPSMQtTeleOperationJSON
                               ${sawRobotIO1394_LIBRARIES}
                               ${sawIntuitiveResearchKit_LIBRARIES}
                               ${sawUDPPSM_LIBRARIES}
                               ${sawControllers_LIBRARIES}
                               ${sawTextToSpeech_LIBRARIES})

        # link against cisst libraries (and dependencies)
        cisst_target_link_libraries (sawUDPPSMQtTeleOperationJSON ${REQUIRED_CISST_LIBRARIES})

        # main program used to configure TeleOperation
        qt_wrap_cpp (sawUDPPSMQtUDPServer sawUDPPSMQtUDPServer_QT_WRAP mainQtUDPServer.h)
        add_executable (sawUDPPSMQtUDPServer
                        mainQtUDPServer.cpp
                        ${sawUDPPSMQtUDPServer_QT_WRAP})
        set_property (TARGET sawUDPPSMQtUDPServer PROPERTY FOLDER "sawUDPPSM")

        # link against cisst libraries (and dependencies)
        cisst_target_link_libraries (sawUDPPSMQtUDPServer ${REQUIRED_CISST_LIBRARIES})

      endif (CISST_HAS_JSON)

    endif (CISST_HAS_QT)

  endif (sawRobotIO1394_FOUND AND sawControllers_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
