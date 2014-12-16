/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Preetham Chalasani
  Created on:

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawUDPPSM/mtsVFResolveRates.h>

CMN_IMPLEMENT_SERVICES(mtsVFResolveRates)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFResolveRates::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    CurrentKinematics  = Kinematics.at(0);
    DesiredKinematics  = Kinematics.at(1);

    // Current Frame
    vctFrm4x4 CurrentFrame;
    CurrentFrame.FromNormalized(CurrentKinematics->Frame);

    // Desired Frame
    vctFrm4x4 DesiredFrame;
    DesiredFrame.FromNormalized(DesiredKinematics->Frame);

    // Put Jacobian into matrix ref
    ObjectiveMatrixRef.Zeros();
    ObjectiveMatrixRef.Diagonal().SetAll(1.0);

    Counter++;
}
