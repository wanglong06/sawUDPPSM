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
    UDPsend(osaSocket::UDP),
    UDPrecv(osaSocket::UDP),
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

        // ZC: Hack
        // Destination: 10005
        // Listen to 10006
        //short portListen = 10006;
        //short portSend = 10005;
        short portSend = port;
        short portListen = port+1;// This is because if for local test use, same port cannot be used for sending and receiving at the same time.

        UDPsend.SetDestination(ip, portSend);
        UDPrecv.AssignPort(portListen);
        SocketConfigured = true;
        UdpEchoRequested = true;
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
    // ZC: HACK
//    SetState(PSM_READY);

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
        double packetSent[17];

        // send new desired position
        if (IsCartesianGoalSet) {
            // Packet format (10 doubles): Message Type, gripper, x, y, z, q0, qx, qy, qz
            // Message Type value table:
            /*  Let us use the integer part of this number as a binary number, ABCD-EFGH
                If H=0, this message is invalid
                If H=1, this message is valid, i.e the desired pose will be accepted by PSM
                If G=0, this message does not request time stamping
                If G=1, this message does request time stamping
            */
            double message_type =1;
            if (UdpEchoRequested)
            {
                const osaTimeServer & timeServer = mtsComponentManager::GetInstance()->GetTimeServer();
                double time = timeServer.GetRelativeTime();
                if (UdpEchoSent) {
                    message_type = message_type;
                } //If sent, just wait for receiving the echo
                else {
                   message_type = message_type + 2;
                   UdpEchoSent = true;
                } // If not sent, send it.
                packetSent[16] = time;
            }
            else
            {
                packetSent[16] = 0;
            }
            packetSent[0] = message_type;
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
            packetSent[9] = qrot.Z();
            packetSent[10] = 0; //  Fx
            packetSent[11] = 0; //  Fy
            packetSent[12] = 0; //  Fz
            packetSent[13] = 0; //  Mx
            packetSent[14] = 0; //  My
            packetSent[15] = 0; //  Mz
            packetSent[17] = CommunicationDelay;
            //UDPsend.Send(reinterpret_cast<char *>(packetSent), sizeof(packetSent));
            UDPsend.Send((char *)packetSent, sizeof(packetSent));
            IsCartesianGoalSet = false;
        }
    }

//    std::cout << "hello world" << std::endl;
}

void mtsUDPPSM::Cleanup(void)
{
    UDPsend.Close();
    UDPrecv.Close();
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
            bytesRead = UDPrecv.Receive(buffer1, sizeof(buffer1), 0.0);
            if (bytesRead>0){
                memcpy(buffer2,buffer1,sizeof(buffer1));
                LatestRead= bytesRead;
            }
        } while (bytesRead);
        if (LatestRead > 0) {
            if (LatestRead == 136) {
                //std::cerr << "*" << std::flush;
                packetReceived = reinterpret_cast<double *>(buffer2);
                // unpack UDP packets
                int message_type;
                message_type = int (packetReceived[0]);
                if(message_type&=2) // if the 2nd bit is true, meaning this is an echo packet
                    {
                        const osaTimeServer & timeServer = mtsComponentManager::GetInstance()->GetTimeServer();
                        double time = timeServer.GetRelativeTime();
                        CommunicationDelay =time - packetReceived[15];
                        UdpEchoSent=false; // reset the sent flag
                        UdpEchoReceived=true;
                    }
                else
                    {
                        UdpEchoReceived=false;
                    }
                vct3 translation;
                translation.Assign(packetReceived[2],
                                   packetReceived[3],
                                   packetReceived[4]);
                vctQuatRot3 qrot;
                qrot.W() = packetReceived[5];
                qrot.X() = packetReceived[6];
                qrot.Y() = packetReceived[7];
                qrot.Z() = packetReceived[8];
                vct3 slave_sensed_force;
                slave_sensed_force.Assign(  packetReceived[9],
                                            packetReceived[10],
                                            packetReceived[11]);
                vct3 slave_sensed_torque;
                slave_sensed_torque.Assign( packetReceived[12],
                                            packetReceived[13],
                                            packetReceived[14]);
                CartesianCurrent.Translation().Assign(translation);
                CartesianCurrent.Rotation().FromNormalized(qrot);
                /*Here some actions needed for the use of force and torque*/
                if (Counter%20 == 0) {
                    std::cout << "CartesianCurrent = " << std::endl
                              << CartesianCurrent << std::endl << std::endl;
                }

            } else {
                std::cerr << "! LatestReading = " << LatestRead << std::endl
                          <<"! Counter = " << Counter <<std::endl << std::flush;
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
