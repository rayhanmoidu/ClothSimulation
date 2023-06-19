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
    void addExternalForce(Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float), ForceType);
private:
    void computeNewParticleStates(StateComputationMode);
    void applyExternalForces();
    
    // particle state computation methods
    void optimizationImplicitEuler();
    void timeStepping();
    
    void evaluateHessian(Eigen::VectorXf);
    void evaluateGradient(Eigen::VectorXf);
    void evaluateMassMatrix();
    Eigen::VectorXf getCurPosition();
    Eigen::VectorXf getPrevPosition();
    
    Eigen::Vector3f applyNewtonsMethod(SpringEndpoint);
    Eigen::VectorXf applyNewtonsMethod_Test();
    
    vector<SpringEndpoint*> particles;
    vector<std::pair<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float), ForceType>> externalForces;
    float time;
    float timeStep;
    Canvas canvas;
    StateComputationMode mode;
    
    Eigen::VectorXf gradient;
    Eigen::MatrixXf hessian;
    Eigen::MatrixXf massMatrix;
    int n;
};

#endif /* Simulation_hpp */
