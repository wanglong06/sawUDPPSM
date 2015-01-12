#include <sawUDPPSM/mtsUDPOptimizer.h>
#include <sawUDPPSM/mtsVFResolveRates.h>

mtsUDPOptimizer::mtsUDPOptimizer(const size_t numOfJoints) :
    BaseType(numOfJoints, 0)
{

}

// Do RR uses jacobian ??
void mtsUDPOptimizer::InitializeRRVF(const size_t rows,
                                     const std::string & vfName,
                                     const std::string & currentKinName,
                                     const std::string & desiredKinName,
                                     const std::string &currentSenName)
{

    // @todo: Initialize the kinematics here
    // Set the Kinematics
    CurrentSlaveKinematics.Name = currentKinName.data();
    DesiredSlaveKinematics.Name = desiredKinName.data();
    CurrentSlaveKinematics.JointState = &CurrentJointState;

    FollowData.Name = vfName.data();
    FollowData.ObjectiveRows = rows;
    FollowData.KinNames.clear();
    FollowData.KinNames.push_back(currentKinName);
    FollowData.KinNames.push_back(desiredKinName);

    AddVFResolveRates(FollowData);

//    CurrentSlaveKinematics.Jacobian.SetSize(rows, NumOfJoints, VCT_COL_MAJOR);
}

void mtsUDPOptimizer::AddVFResolveRates(const mtsVFDataBase &vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!this->SetVFData(vf, typeid(mtsVFResolveRates)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFResolveRates *>(vf.Name,new mtsVFResolveRates(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
    }
}

