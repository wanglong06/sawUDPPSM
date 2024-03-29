/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mainQtUDPServer.cpp 4588 2013-12-04 22:53:51Z adeguet1 $

  Author(s):  Peter Kazanzides, Anton Deguet
  Created on: 2009

  (C) Copyright 2009-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "mainQtUDPServer.h"
#include <string.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnCommandLineOptions.h>

#include <QApplication>

int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("osaSocket", CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    cmnLogger::SetMaskClass("osaSocketServer", CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    unsigned short port = 0;

    cmnCommandLineOptions options;
    options.AddOptionOneValue("p", "port",
                              "UDP Port", cmnCommandLineOptions::REQUIRED_OPTION, &port);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    QApplication application(argc, argv);

    UDPQtSocket socket;
    socket.AssignPort(port);
    socket.show();

    application.exec();

    socket.Close();

    return 0;
}
