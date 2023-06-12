//
//  Simulation.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-04.
//

#ifndef Simulation_hpp
#define Simulation_hpp

#include <stdio.h>
#include "SpringEndpoint.hpp"
#include "Canvas.hpp"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

enum StateComputationMode {
    TIMESTEP = 0,
    OPTIMIZATION_IMPLICIT_EULER,
};

class Simulation {
public:
    Simulation(float, vector<SpringEndpoint*>, Canvas, StateComputationMode);
    void update();
    void addExternalForce(Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float));
private:
    void computeNewParticleStates(StateComputationMode);
    void applyExternalForces();
    
    // particle state computation methods
    void optimizationImplicitEuler();
    void timeStepping();
    
    Eigen::Vector3f applyNewtonsMethod(SpringEndpoint);
    
    vector<SpringEndpoint*> particles;
    vector<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float)> externalForces;
    float time;
    float timeStep;
    Canvas canvas;
    StateComputationMode mode;
};

#endif /* Simulation_hpp */
