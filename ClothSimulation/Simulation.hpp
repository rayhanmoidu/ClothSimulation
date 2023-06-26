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
#include "Examples.hpp"

enum StateComputationMode {
    TIMESTEP = 0,
    OPTIMIZATION_IMPLICIT_EULER,
//    OPTIMIZATION_IMPLICIT_EULER_BY_PARTICLE,
};

class Simulation {
public:
    Simulation(ClothRepresentation, float, Canvas, StateComputationMode);
    void update();
    void addExternalForce(Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float), ForceType);
    
private:
    void computeNewParticleStates(StateComputationMode);
    void applyExternalForces();
    
    Eigen::Vector3f calculateSpringForce(Eigen::Vector3f p1, Eigen::Vector3f p2, float r, float k);
    
    bool isParticleFixed(SpringEndpoint*);
    
    void computeNeighbourRelationships();
    
    // particle state computation methods
    void optimizationImplicitEuler();
    void timeStepping();
    
    void evaluateHessian(Eigen::VectorXf);
    Eigen::MatrixXf evaluateHessian_Portion(Eigen::Vector3f p1, Eigen::Vector3f p2);
    void evaluateGradient(Eigen::VectorXf);
    void evaluateMassMatrix();
    Eigen::VectorXf getCurPosition();
    Eigen::VectorXf getPrevPosition();
    
    Eigen::VectorXf applyNewtonsMethod();
    
    vector<SpringEndpoint*> particles;
    std::vector<Spring> springs;
    vector<std::pair<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float), ForceType>> externalForces;
    float time;
    float timeStep;
    Canvas canvas;
    StateComputationMode mode;
    
    vector<int> fixedIds;
    
    Eigen::VectorXf gradient;
    Eigen::MatrixXf hessian;
    Eigen::MatrixXf massMatrix;
    int n;
};

#endif /* Simulation_hpp */
