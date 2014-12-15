#ifndef MTSUDPOPTIMIZER_H
#define MTSUDPOPTIMIZER_H

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitOptimizer.h>

class mtsUDPOptimizer: public mtsIntuitiveResearchKitOptimizer
{
public:

    typedef mtsIntuitiveResearchKitOptimizer BaseType;

    mtsUDPOptimizer() {};
    mtsUDPOptimizer(const size_t numOfJoints, robManipulator *manip);
};

#endif // MTSUDPOPTIMIZER_H
