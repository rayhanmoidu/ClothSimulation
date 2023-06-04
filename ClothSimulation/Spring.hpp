//
//  Spring.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#ifndef Spring_hpp
#define Spring_hpp

#include <stdio.h>
#include "Particle.hpp"
#include <vector>

class Spring {
public:
    Spring(Particle, Particle, float, float, float);
    void applyInternalSpringForces();
    void applyAdditionalForce(Eigen::Array3f (*)(Particle));
    void stepForward(float);
    
    vector<Particle> getParticles();
private:
    float restLength;
    float springConstant;
    float dampingConstant;
    Particle p1;
    Particle p2;
};

#endif /* Spring_hpp */
