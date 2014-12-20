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

    // Incremental Frame
    vctFrm4x4 Pose_dx = CurrentFrame.Inverse() * DesiredFrame;

    // Current position error
    vctDoubleVec ep;
    ep.Assign(DesiredFrame.Translation() - CurrentFrame.Translation());

    // Known variables
    vctDoubleVec vMin(3,0.0);
    vMin = ep.Normalized();
    double epsilon_p, alpha, lambda, vMax;
    alpha = (lambda * epsilon_p)/ vMax;

    // Translation Part
    vct3 dx_translation = (ep-vMin).Multiply((signbit(ep.Norm() - epsilon_p)));

    // Rotation Part
    vct3 dx_rotation;
    vctAxAnRot3 dxRot;
    vct3 dxRotVec;
    dxRot.FromNormalized(Pose_dx.Rotation());
    dxRotVec = dxRot.Axis() * dxRot.Angle();
    dx_rotation[0] = dxRotVec[0];
    dx_rotation[1] = dxRotVec[1];
    dx_rotation[2] = dxRotVec[2];
    dx_rotation = CurrentFrame.Rotation() * dx_rotation;

    vctDoubleVec dx(6);
    std::copy(dx_translation.begin(), dx_translation.end(), dx.begin()  );
    std::copy(dx_rotation.begin()   , dx_rotation.end()   , dx.begin()+3);
    ObjectiveVectorRef.Assign(dx);

    // Put Jacobian into matrix ref
    ObjectiveMatrixRef.Zeros();
    ObjectiveMatrixRef.Diagonal().SetAll(1.0);

    Counter++;
}
