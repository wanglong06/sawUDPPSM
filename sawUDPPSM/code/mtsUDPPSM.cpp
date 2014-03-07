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
        /*  Long Wang:
            Please refer to the definition in osaSocket.h
            For a server and a client, you have to do
            server.AssignPort(serverPort);
            client.SetDestination(serverHost, serverPort);
            In our case, the socket is server and client at the same time.
        */
        Socket.SetDestination(ip, port);
        Socket.AssignPort(port);
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

    if (SocketConfigured) {
        double packetSent[9];

        // send new desired position
        if (IsCartesianGoalSet) {
            // Packet format (9 doubles): buttons (clutch, coag), gripper, x, y, z, q0, qx, qy, qz
            // For the buttons: 0=None
            packetSent[0] = 1; // This bit will tell PSM if this message is valid or not
            packetSent[1] = DesiredOpenAngle;
            vct3 pos = CartesianGoalSet.Goal().Translation();
            packetSent[2] = pos.X();
            packetSent[3] = pos.Y();
            packetSent[4] = pos.Z();
            vctQuatRot3 qrot(CartesianGoalSet.Goal().Rotation());
            packetSent[5] = qrot.W();
            packetSent[6] = qrot.X();
            packetSent[7] = qrot.Y();
            packetSent[8] = qrot.Z();
            //Socket.Send(reinterpret_cast<char *>(packetSent), sizeof(packetSent));
            Socket.Send((char *)packetSent, sizeof(packetSent));
            IsCartesianGoalSet = false;
        }
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
        // read position and orientation from slave
        // read UDP packets
        int LatestRead = 0;
        int bytesRead = 1;
        char buffer1[512];
        char buffer2[512];
        double * packetReceived;
        // flush out the buffer
        do {
            bytesRead = Socket.Receive(buffer1, sizeof(buffer1), 0.0);
            if (bytesRead>0){
                memcpy(buffer2,buffer1,sizeof(buffer1));
                LatestRead= bytesRead;
                }
            } while (bytesRead);
        if (LatestRead > 0) {
            if (LatestRead == 9 * sizeof(double)) {
                packetReceived = reinterpret_cast<double *>(buffer2);
                // unpack UDP packets
                vct3 translation;
                translation.Assign(packetReceived[2],
                                   packetReceived[3],
                                   packetReceived[4]);
                vctQuatRot3 qrot;
                qrot.W() = packetReceived[5];
                qrot.X() = packetReceived[6];
                qrot.Y() = packetReceived[7];
                qrot.Z() = packetReceived[8];
                CartesianCurrent.Translation().Assign(translation);
                CartesianCurrent.Rotation().FromNormalized(qrot);
            } else {
                std::cerr << "!" << std::flush;
            }
        } else {
            std::cerr << "~" << std::flush;
        }
    } else {
        // for state not ready
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
