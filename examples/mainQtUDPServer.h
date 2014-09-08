/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mainQtUDPServer.h 4588 2013-12-04 22:53:51Z adeguet1 $

  Author(s):  Peter Kazanzides, Anton Deguet
  Created on: 2009

  (C) Copyright 2009-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaSocket.h>

#include <QWidget>
#include <cisstVector/vctQtWidgetRotation.h>

// simple wrapper around socket to provide a read method tied to Qt
// timer
class UDPQtSocket: public QWidget {
    Q_OBJECT;
    osaSocket mSocket;
    vctQtWidgetRotationDoubleRead * RotationWidget;
public:
    inline UDPQtSocket(void):
        mSocket(osaSocket::UDP)
    {
        startTimer(1); // in ms
        RotationWidget = new vctQtWidgetRotationDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);

    }

    inline void AssignPort(unsigned short port) {
        while (!mSocket.AssignPort(port)) {
            std::cout << "Unable to connect to port, will try again in 5 seconds." << std::endl;
            osaSleep(5.0 * cmn_s);
        }
        std::cout << std::endl
                  << "Started server on "
                  << osaSocket::GetLocalhostIP() << ":" << port << std::endl
                  << std::endl;
    }

    inline void Close(void) {
        mSocket.Close();
    }

    void show(void) {
        RotationWidget->show();
    }


private slots:
    void timerEvent(QTimerEvent * CMN_UNUSED(event)) {
        char buffer[512];
        int bytesRead;
        // receive
        bytesRead = mSocket.Receive(buffer, sizeof(buffer), 0.0);
        if (bytesRead > 0) {
            if (bytesRead == 72) {
                vctQuatRot3 qrot;
                double * bufferDouble = reinterpret_cast<double *>(buffer); 
                qrot.W() = bufferDouble[5];
                qrot.X() = bufferDouble[6];
                qrot.Y() = bufferDouble[7];
                qrot.Z() = bufferDouble[8];
                RotationWidget->SetValue(vctMatRot3(qrot));
            } else {
                std::cerr << " -> ERROR: packet size should be 72, got " << bytesRead << std::endl;
            }

            double packetSent[9];
            // Packet format (9 doubles): buttons (clutch, coag), gripper, x, y, z, q0, qx, qy, qz
            // For the buttons: 0=None
            packetSent[0] = 0.0;
            packetSent[1] = 3.14; // DesiredOpenAngle;
            // vct3 pos = CartesianGoalSet.Goal().Translation();
            packetSent[2] = 10.0; // pos.X();
            packetSent[3] = 10.0; // pos.Y();
            packetSent[4] = 10.0; // pos.Z();
            // vctQuatRot3 qrot(CartesianGoalSet.Goal().Rotation());
            packetSent[5] = 10.0; // qrot.W();
            packetSent[6] = 10.0; // qrot.X();
            packetSent[7] = 10.0; // qrot.Y();
            packetSent[8] = 10.0; // qrot.Z();
            mSocket.Send(reinterpret_cast<char *>(packetSent), sizeof(packetSent));
        }
    }
};
