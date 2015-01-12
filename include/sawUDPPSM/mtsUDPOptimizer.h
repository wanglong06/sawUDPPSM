#ifndef MTSUDPOPTIMIZER_H
#define MTSUDPOPTIMIZER_H

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitOptimizer.h>

class mtsUDPOptimizer: public mtsIntuitiveResearchKitOptimizer
{
public:

    typedef mtsIntuitiveResearchKitOptimizer BaseType;

    mtsUDPOptimizer() {};
    mtsUDPOptimizer(const size_t numOfJoints);

    void InitializeRRVF(const size_t rows,
                        const std::string & vfName,
                        const std::string & currentKinName,
                        const std::string & desiredKinName,
                        const std::string &currentSenName);

    void AddVFResolveRates(const mtsVFDataBase & vf);

    void UpdateParams(const double tickTime,
                      const vctFrm4x4 cartesianCurrent,
                      const vctFrm4x4 cartesianDesired);
};

#endif // MTSUDPOPTIMIZER_H
