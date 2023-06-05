//
//  Simulation.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-04.
//

#ifndef Simulation_hpp
#define Simulation_hpp

#include <stdio.h>
#include "Particle.hpp"
#include "Canvas.hpp"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class Simulation {
public:
    Simulation(float, vector<Particle>, Canvas);
    void update();
    void addExternalForce(Eigen::Vector3f (*)(Particle, float));
private:
    void computeNewParticleStates();
    void applyExternalForces();
    
    vector<Particle> particles;
    vector<Eigen::Vector3f (*)(Particle, float)> externalForces;
    float time;
    float timeStep;
    Canvas canvas;
};

#endif /* Simulation_hpp */
