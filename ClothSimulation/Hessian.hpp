//
//  Hessian.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-07-10.
//

#ifndef Hessian_hpp
#define Hessian_hpp

#include <stdio.h>
#include "Spring.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "Examples.hpp"
#include <eigen3/Eigen/Sparse>

class Hessian {
public:
    Hessian();
    Eigen::VectorXf computeHessian_dfx(Eigen::Vector3f p, Eigen::Vector3f q, float r, float k);
    Eigen::VectorXf computeHessian_dfy(Eigen::Vector3f p, Eigen::Vector3f q, float r, float k);
    Eigen::VectorXf computeHessian_dfz(Eigen::Vector3f p, Eigen::Vector3f q, float r, float k);
};

#endif /* Hessian_hpp */
