//
//  Spring.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#ifndef Spring_hpp
#define Spring_hpp

#include <stdio.h>
#include "SpringEndpoint.hpp"
#include <vector>

class Spring {
public:
    Spring(SpringEndpoint, SpringEndpoint, float, float, float);
    void applyInternalSpringForces();
    void applyAdditionalForce(Eigen::Vector3f (*)(SpringEndpoint));
    void stepForward(float);
    
    vector<SpringEndpoint> getParticles();
private:
    float restLength;
    float springConstant;
    float dampingConstant;
    SpringEndpoint p1;
    SpringEndpoint p2;
};

#endif /* Spring_hpp */
