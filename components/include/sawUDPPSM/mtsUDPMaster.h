#ifndef _mtsUDPMaster_h
#define _mtsUDPMaster_h

#include <cisstOSAbstraction/osaSocket.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robQuintic.h>
#include <cisstVector/vctQuaternion.h>
//

#include <cisstOSAbstraction/osaTimeServer.h>

#define PACKETSIZE 22

class mtsUDPMaster: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsUDPMaster(const std::string & componentName, const double periodInSeconds,
                 const std::string & ip, const unsigned int port);
    mtsUDPMaster(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsUDPMaster();

    void Configure(const std::string & filename = ""){}
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:
    void Init(void);

    enum RobotStateType {
        MASTER_UNINITIALIZED, /*! State when constructed */
        MASTER_READY,
        MASTER_POSITION_CARTESIAN, /**< Go to command cartesian position */
        MASTER_MANUAL /**< User manually move robot */
    };

    /*! Get data from the PID level based on current state. */
    void GetRobotData(void);

    // Get command from UDP 
    void GetUDPCommand(void);

    // Send feedback through UDP
    void SendUDPData(void);

    // Write the command to the robot
    void SendCommandToRobot(void);

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite RobotState;
    } EventTriggers;

    // Functions for ATI
    struct InterfaceForceTorque{
        mtsFunctionRead GetData;
    } NetFT;

    class RobotPSM {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionWrite SetJawPosition;

        mtsFunctionRead  GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianSet;
        vctFrm4x4 CartesianPrevious;
    };
    RobotPSM * mPSM;
    /*
    // ZC: cache Cartesian Goal posiiton
    prmPositionCartesianSet CartesianGoalSet;
    bool IsCartesianGoalSet;

    prmPositionCartesianGet CartesianCurrentParam;
    vctFrm4x4 CartesianCurrent;
    vctFrm4x4 CartesianPrevious;

    vctFrm4x4 CartesianPositionFrm;
    
    RobotStateType RobotState;
    */

    bool CommandEnabled;

    double DesiredOpenAngle;
    double CurrentOpenAngle;

    std::string RobotState;

    vctFrm4x4 CartesianCurrent;
    vctFrm4x4 CartesianDesired;

    osaSocket UDPsend;
    osaSocket UDPrecv;
    bool SocketConfigured;

    //By Long Wang
    double CommunicationDelay;
    bool UdpEchoRequested;
    bool UdpEchoSent;
    bool UdpEchoReceived;
    double packetSent[PACKETSIZE];

    // ATI force sensor
    mtsDoubleVec mtsRawSensorData;

    int Counter;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsUDPMaster);

#endif // _mtsUDPMaster_h
