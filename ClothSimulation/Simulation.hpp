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
    OPTIMIZATION_IMPLICIT_EULER_BY_PARTICLE,
};

class Simulation {
public:
    Simulation(float, vector<Spring>, vector<SpringEndpoint*>, Canvas, StateComputationMode);
    void update();
    void addExternalForce(Eigen::Vector3f (*)(SpringEndpoint), ForceType);
private:
    void computeNewParticleStates(StateComputationMode);
    void applyForces();
    Eigen::Vector3f calculateSpringForce(Eigen::Vector3f p1, Eigen::Vector3f p2, float r, float k);
    
    void getParticlesFromSprings();
    
    // particle state computation methods
    void optimizationImplicitEuler();
//    void optimizationImplicitEuler_ByParticle();
    void timeStepping();
    
    void evaluateHessian(Eigen::VectorXf);
    Eigen::MatrixXf evaluateHessian_Portion(Eigen::Vector3f, Eigen::Vector3f);
    void evaluateGradient(Eigen::VectorXf);
    void evaluateMassMatrix();
    Eigen::VectorXf getCurPosition();
    Eigen::VectorXf getPrevPosition();
    
//    Eigen::Vector3f applyNewtonsMethod(SpringEndpoint);
    Eigen::VectorXf applyNewtonsMethod_Test();
    
    vector<Spring> springs;
    vector<SpringEndpoint*> particles;
    
    vector<std::pair<Eigen::Vector3f (*)(SpringEndpoint), ForceType>> externalForces;
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
