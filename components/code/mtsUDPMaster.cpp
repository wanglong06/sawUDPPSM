// system include
#include <iostream>
#include <time.h>

// cisst
#include <sawUDPPSM/mtsUDPMaster.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsUDPMaster, mtsTaskPeriodic);

mtsUDPMaster::mtsUDPMaster(const std::string & componentName, const double periodInSeconds,
                           const std::string & ip, const unsigned int port):
    mtsTaskPeriodic(componentName, periodInSeconds),
    UDPsend(osaSocket::UDP),
    UDPrecv(osaSocket::UDP),
    SocketConfigured(false),
    Counter(0),
    mPSM(0)
{
    Init();


    /*
    SetState(MASTER_UNINITIALIZED);
    DesiredOpenAngle = 0 * cmnPI_180;
    this->StateTable.AddData(CartesianCurrentParam, "CartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrentParam, "GetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsUDPMaster::SetPositionCartesian, this, "SetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsUDPMaster::SetRobotControlState, this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.Status, "Status", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.Warning, "Warning", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.Error, "Error", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotState, "RobotState", std::string(""));
        // Nico added these
        interfaceProvided->AddCommandWrite(&mtsUDPMaster::SetOpenAngle, this, "SetOpenAngle");
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats, "GetPeriodStatistics");
    }
    */
    // Initializing packet to be zeroes
    for (int i = 0; i < PACKETSIZE; ++i) {
        packetSent[i] = 0;
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
        
        short portListen = port;// This is because if for local test use, same port cannot be used for sending and receiving at the same time.
        short portSend = port+1;

        UDPsend.SetDestination(ip, portSend);
        UDPrecv.AssignPort(portListen);
        SocketConfigured = true;
        UdpEchoRequested = true;
    }
}

void mtsUDPMaster::Init(void)
{
    if (!mPSM) {
        mPSM = new RobotPSM;
    }

    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("PSM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mPSM->GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", mPSM->SetPositionCartesian);
        interfaceRequired->AddFunction("SetJawPosition", mPSM->SetJawPosition);
        interfaceRequired->AddFunction("GetRobotControlState", mPSM->GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState", mPSM->SetRobotControlState);        
        //interfaceRequired->AddEventHandlerWrite(&mtsUDPMaster::PSMErrorEventHandler,
        //                                        this, "Error");
    }
    
    // ATI Net Force/Torque Sensor - Needs connection
    interfaceRequired = BaseType::AddInterfaceRequired("RequiresATINetFTSensor", MTS_OPTIONAL);
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetFTData", NetFT.GetData);
    }

    CommandEnabled = false;
}

void mtsUDPMaster::Startup(void)
{
    //this->SetState(MASTER_UNINITIALIZED);
}

void mtsUDPMaster::Run(void)
{
    ProcessQueuedEvents();

    GetRobotData();
    GetUDPCommand();

    ProcessQueuedCommands();
    SendCommandToRobot();

    SendUDPData();
    // ZC: HACK
    //    SetState(MASTER_READY);
    /*
    Counter++;

    ProcessQueuedEvents();
    ProcessQueuedCommands();
    GetRobotData();

    switch (RobotState) {
    case MASTER_UNINITIALIZED:
        break;
    case MASTER_READY:
    case MASTER_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case MASTER_MANUAL:
        break;
    default:
        break;
    }

    RunEvent();
    ProcessQueuedCommands();
    */
}

void mtsUDPMaster::Cleanup(void)
{
    UDPsend.Close();
    UDPrecv.Close();
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsUDPMaster::GetRobotData(void)
{
     mPSM->GetPositionCartesian(mPSM->PositionCartesianCurrent);
     CartesianCurrent.FromNormalized(mPSM->PositionCartesianCurrent.Position());     
     mPSM->GetRobotControlState(RobotState);
     NetFT.GetData(mtsRawSensorData);
}

void mtsUDPMaster::GetUDPCommand(void)
{
    int bytesRead;
    char buffer[512];
    double * packetReceived;
    
    bytesRead = UDPrecv.Receive(buffer, sizeof(buffer), 10*cmn_ms);
    
    if(bytesRead > 0) {
        if(bytesRead == 22*8) {
            packetReceived = reinterpret_cast<double *>(buffer);

            // unpack UDP packets
            double command_enabled = packetReceived[0];
            if(command_enabled>0.5) {
                CommandEnabled = true;
            } else {
                CommandEnabled = false;
            }
 
            // if bit_0 is true, meaning command is enabled
            double enabled = packetReceived[0];
            
            DesiredOpenAngle = packetReceived[1];
            vct3 translation;
            translation.Assign(packetReceived[2],
                               packetReceived[3],
                               packetReceived[4]);
            vctMatRot3 rotation;
            rotation(0,0) = packetReceived[5];
            rotation(1,0) = packetReceived[6];
            rotation(2,0) = packetReceived[7];
            rotation(0,1) = packetReceived[8];
            rotation(1,1) = packetReceived[9];
            rotation(2,1) = packetReceived[10];
            rotation(0,2) = packetReceived[11];
            rotation(1,2) = packetReceived[12];
            rotation(2,2) = packetReceived[13];

            CartesianDesired.Translation().Assign(translation);
            CartesianDesired.Rotation().FromNormalized(rotation);

            // UDP echo to keep track the communication delay time
            int message_type;
            message_type = int (packetReceived[21]);

            // if bit_1 is true, meaning this is an echo packet
            if(message_type&=2) {
                const osaTimeServer & timeServer = mtsComponentManager::GetInstance()->GetTimeServer();
                double time = timeServer.GetRelativeTime();
                CommunicationDelay = time - packetReceived[20];
                UdpEchoSent=false; // reset the sent flag
                UdpEchoReceived=true;
            } else {
                UdpEchoReceived=false;
            }
 
            // Here some actions needed for the use of force and torque
            if (Counter%200 == 0) {
                /*std::cerr << "CartesianDesired = " << std::endl
                          << CartesianDesired << std::endl;
                          */
            }
        } else {
            std::cerr << "Bytes Read : " << bytesRead << std::endl;
            packetReceived = reinterpret_cast<double *>(buffer);
            for (int i = 0; i < bytesRead; ++i) {
                std::cerr << packetReceived[i] << " ";
            }
            std::cerr << std::endl;
        }
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "GetReadings: UDP receive failed" << std::endl;
    }
}

void mtsUDPMaster::SendCommandToRobot(void)
{
    if (CommandEnabled){
        // convert to prm type
        mPSM->SetRobotControlState(mtsStdString("Cartesian position"));
        mPSM->PositionCartesianSet.Goal().From(CartesianDesired);
        mPSM->SetPositionCartesian(mPSM->PositionCartesianSet);
    }else{
    }
}

void mtsUDPMaster::SendUDPData(void)
{

    // Packet format (10 doubles): Message Type, gripper, x, y, z, q0, qx, qy, qz
    // Message Type value table:
    //  Let us use the integer part of this number as a binary number, ABCD-EFGH
    //          If H=0, this message is invalid
    //          If H=1, this message is valid, i.e the desired pose will be accepted by MTM
    //          If G=0, this message does not request time stamping
    //          If G=1, this message does request time stamping

    double message_type =1;
    if (UdpEchoRequested) {
        const osaTimeServer & timeServer = mtsComponentManager::GetInstance()->GetTimeServer();
        double time = timeServer.GetRelativeTime();
        if (UdpEchoSent) {
            message_type = message_type;
        } //If sent, just wait for receiving the echo
        else {
            message_type = message_type + 2;
            UdpEchoSent = true;
        } // If not sent, send it.
        packetSent[20] = time;
    } else {
        packetSent[20] = 0;
    }

    vctMatRot3 rotation;
    rotation.FromNormalized(CartesianCurrent.Rotation());
    packetSent[0] = message_type;
    packetSent[1] = CurrentOpenAngle;
    vct3 pos = CartesianCurrent.Translation();
    packetSent[2] = pos.X();
    packetSent[3] = pos.Y();
    packetSent[4] = pos.Z();

    packetSent[5] = rotation(0,0);
    packetSent[6] = rotation(1,0);
    packetSent[7] = rotation(2,0);
    packetSent[8] = rotation(0,1);
    packetSent[9] = rotation(1,1);
    packetSent[10] = rotation(2,1);
    packetSent[11] = rotation(0,2);
    packetSent[12] = rotation(1,2);
    packetSent[13] = rotation(2,2);
    
    packetSent[14] = mtsRawSensorData[0];
    packetSent[15] = mtsRawSensorData[1];
    packetSent[16] = mtsRawSensorData[2];
    packetSent[17] = mtsRawSensorData[3];
    packetSent[18] = mtsRawSensorData[4];
    packetSent[19] = mtsRawSensorData[5];
    

    packetSent[21] = CommunicationDelay;

    if(SocketConfigured) {
        UDPsend.Send((char *)packetSent, sizeof(packetSent));
    } else {
        CMN_LOG_CLASS_RUN_ERROR << "RunPositionCartesian: Socket Not Configured." << std::endl;
    }
}


mtsUDPMaster::~mtsUDPMaster()
{
    if (mPSM) {
        delete mPSM;
    }
}