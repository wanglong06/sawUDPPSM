/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsUDPPSM.h 4687 2014-02-09 20:50:35Z zchen24 $

  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsUDPPSM_h
#define _mtsUDPPSM_h

#include <cisstOSAbstraction/osaSocket.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robQuintic.h>

class mtsUDPPSM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsUDPPSM(const std::string & componentName, const double periodInSeconds,
              const std::string & ip, const unsigned int port);
    mtsUDPPSM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsUDPPSM() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    enum RobotStateType {
        PSM_UNINITIALIZED, /*! State when constructed */
        PSM_READY,
        PSM_POSITION_CARTESIAN, /**< Go to command cartesian position */
        PSM_MANUAL /**< User manually move robot */
    };

    /*! Get data from the PID level based on current state. */
    void GetRobotData(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const RobotStateType & newState);

    /*! Cartesian state. */
    void RunPositionCartesian(void);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetOpenAngle(const double & openAngle);
    void SetRobotControlState(const std::string & state);

    // Functions for events
    struct {
        mtsFunctionWrite RobotStatusMsg;
        mtsFunctionWrite RobotErrorMsg;
        mtsFunctionWrite ManipClutch;
        mtsFunctionWrite SUJClutch;
    } EventTriggers;

    // ZC: cache Cartesian Goal posiiton
    prmPositionCartesianSet CartesianGoalSet;
    bool IsCartesianGoalSet;

    prmPositionCartesianGet CartesianCurrentParam;
    vctFrm4x4 CartesianCurrent;
    vctFrm4x4 CartesianPrevious;

    vctFrm4x4 CartesianPositionFrm;
    double DesiredOpenAngle;
    RobotStateType RobotState;

    int Counter;

    osaSocket Socket;
    bool SocketConfigured;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsUDPPSM);

#endif // _mtsUDPPSM_h
