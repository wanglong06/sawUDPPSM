#ifndef MTSVFRESOLVERATES_H
#define MTSVFRESOLVERATES_H

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/prmSensorState.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFCartVel.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFResolveRates: A class that contains logic for the implementation of virtual fixtures
 */
class mtsVFResolveRates : public mtsVFCartesianTranslation
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public:

    /*! Constructor */
        mtsVFResolveRates() : mtsVFCartesianTranslation() {}

    /*! Constructor
    \param name String name of object
    */
    mtsVFResolveRates(const std::string & name, mtsVFDataBase * data) : mtsVFCartesianTranslation(name,data){}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);
private:
    prmKinematicsState* CurrentKinematics;
    prmKinematicsState* DesiredKinematics;

    int Counter;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFResolveRates)
#endif // MTSVFRESOLVERATES_H
