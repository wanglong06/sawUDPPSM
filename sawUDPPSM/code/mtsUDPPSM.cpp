/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsIntuitiveResearchKitPSM.cpp 4687 2014-02-09 20:50:35Z zchen24 $

  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>
#include <time.h>

// cisst
#include <sawUDPPSM/mtsUDPPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>


CMN_IMPLEMENT_SERVICES_DERIVED(mtsUDPPSM, mtsTaskPeriodic);

mtsUDPPSM::mtsUDPPSM(const std::string & componentName, const double periodInSeconds,
                     const std::string & ip, const unsigned int port):
    mtsTaskPeriodic(componentName, periodInSeconds),
    Socket(osaSocket::UDP),
    SocketConfigured(false),
    IsCartesianGoalSet(false),
    Counter(0)
{


    SetState(PSM_UNINITIALIZED);
    DesiredOpenAngle = 0 * cmnPI_180;

    this->StateTable.AddData(CartesianCurrentParam, "CartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrentParam, "GetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsUDPPSM::SetPositionCartesian, this, "SetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsUDPPSM::SetOpenAngle, this, "SetOpenAngle");

        interfaceProvided->AddCommandWrite(&mtsUDPPSM::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.ManipClutch, "ManipClutchBtn", prmEventButton());
        interfaceProvided->AddEventWrite(EventTriggers.SUJClutch, "SUJClutchBtn", prmEventButton());
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }

    if (!ip.empty()) {
        Socket.SetDestination(ip, port);
        SocketConfigured = true;
    }
}

void mtsUDPPSM::Configure(const std::string & filename)
{
}

void mtsUDPPSM::Startup(void)
{
    this->SetState(PSM_UNINITIALIZED);
}

void mtsUDPPSM::Run(void)
{
    Counter++;

    ProcessQueuedEvents();
    GetRobotData();

    switch (RobotState) {
    case PSM_UNINITIALIZED:
        break;
    case PSM_READY:
    case PSM_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case PSM_MANUAL:
        break;
    default:
        break;
    }

    RunEvent();
    ProcessQueuedCommands();

    if (SocketConfigured && IsCartesianGoalSet) {
        // Packet format (9 doubles): buttons (clutch, coag), gripper, x, y, z, q0, qx, qy, qz
        // For the buttons: 0=None
        double packet[9];
        packet[0] = 0.0;
        packet[1] = DesiredOpenAngle;
        vct3 pos = CartesianGoalSet.Goal().Translation();
        packet[2] = pos.X();
        packet[3] = pos.Y();
        packet[4] = pos.Z();
        vctQuatRot3 qrot(CartesianGoalSet.Goal().Rotation());
        packet[5] = qrot.W();
        packet[6] = qrot.X();
        packet[7] = qrot.Y();
        packet[8] = qrot.Z();
        Socket.Send((char *)packet, sizeof(packet));
        IsCartesianGoalSet = false;
    }
}

void mtsUDPPSM::Cleanup(void)
{
    Socket.Close();
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsUDPPSM::GetRobotData(void)
{
    if (this->RobotState >= PSM_READY) {
        // perfect slave, does what was asked
        CartesianCurrent.From(CartesianGoalSet.Goal());
    } else {
        CartesianCurrent.Assign(vctFrm4x4::Identity());
    }
    CartesianCurrentParam.Position().From(CartesianCurrent);
}

void mtsUDPPSM::SetState(const RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state " << newState << std::endl;

    switch (newState) {

    case PSM_UNINITIALIZED:
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " not initialized");
        break;

    case PSM_READY:
        // when returning from manual mode, need to re-enable PID
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " ready");
        break;

    case PSM_POSITION_CARTESIAN:
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " position cartesian");
        break;

    case PSM_MANUAL:
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " in manual mode");
        break;
    default:
        break;
    }   
}

void mtsUDPPSM::RunPositionCartesian(void)
{
    //! \todo: should prevent user to go to close to RCM!

    if (IsCartesianGoalSet == true) {
        // compute desired slave position
        CartesianPositionFrm.From(CartesianGoalSet.Goal());

        // reset flag
        IsCartesianGoalSet = false;
    }
}

void mtsUDPPSM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotState == PSM_POSITION_CARTESIAN) {
        CartesianGoalSet = newPosition;
        IsCartesianGoalSet = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionCartesian: PSM not ready" << std::endl;
    }
}

void mtsUDPPSM::SetOpenAngle(const double & openAngle)
{
    DesiredOpenAngle = openAngle;
    IsCartesianGoalSet = true;
}

void mtsUDPPSM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        // SetState(PSM_HOMING_POWERING);
        std::cerr << CMN_LOG_DETAILS << " what should happen when we ask to home?   just set state to ready?" << std::endl;
        SetState(PSM_READY);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(PSM_POSITION_CARTESIAN);
    } else if (state == "Manual") {
        SetState(PSM_MANUAL);
    } else {
        EventTriggers.RobotErrorMsg(this->GetName() + ": unsupported state " + state);
    }
}
