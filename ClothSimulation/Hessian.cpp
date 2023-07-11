//
//  Hessian.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-07-10.
//

#include "Hessian.hpp"

Hessian::Hessian() {
    
}

Eigen::VectorXf Hessian::computeHessian_dfx(Eigen::Vector3f p, Eigen::Vector3f q, float r, float k) {
    float dx = abs(p[0] - q[0]);
    float dy = abs(p[1] - q[1]);
    float dz = abs(p[2] - q[2]);
    
    float d = sqrt(dx*dx + dy*dy + dz*dz);
    
    float acos2 = pow(acos(dy / dx), 2);
    float asec2 = 1/acos2;
    
    float a_tan = atan(dy / dx);
    float sin_atan = sin(a_tan);
    
    float longVal = (k/r * d) - k;
    
    // dpx
    float dpx_clause1 = (asec2 * sin_atan * dy * longVal) / (dx*dx);
    float dpx_clause2 = (k/r * cos(a_tan) * dx) / d;
    float dpx = dpx_clause1 + dpx_clause2;
    
    // dpy
    float dpy_clause1 = (asec2 * sin_atan * longVal) / (dx);
    float dpy_clause2 = (k/r * cos(a_tan) * dy) / d;
    float dpy = -dpy_clause1 + dpy_clause2;
    
    // dpz
    float dpz = (k/r * cos(a_tan) * dz) / d;
    
    // dqx
    float dqx = -dpx_clause1 - dpx_clause2;
    
    // dqy
    float dqy = dpy_clause1 - dpy_clause2;
    
    // dqz
    float dqz = -dpz;
    
    Eigen::VectorXf retVal(6);
    retVal[0] = dpx;
    retVal[1] = dpy;
    retVal[2] = dpz;
    retVal[3] = dqx;
    retVal[4] = dqy;
    retVal[5] = dqz;
    return retVal;
}

Eigen::VectorXf Hessian::computeHessian_dfy(Eigen::Vector3f p, Eigen::Vector3f q, float r, float k) {
    float dx = abs(p[0] - q[0]);
    float dy = abs(p[1] - q[1]);
    float dz = abs(p[2] - q[2]);
    
    float d = sqrt(dx*dx + dy*dy + dz*dz);
    
    float acos2 = pow(acos(dy / dx), 2);
    float asec2 = 1/acos2;
    
    float a_tan = atan(dy / dx);
    float cos_atan = cos(a_tan);
    
    float longVal = (k/r * d) - k;
    
    // dpx
    float dpx_clause1 = (asec2 * cos_atan * dy * longVal) / (dx*dx);
    float dpx_clause2 = (k/r * sin(a_tan) * dx) / d;
    float dpx = -dpx_clause1 + dpx_clause2;
    
    // dpy
    float dpy_clause1 = (asec2 * cos_atan * longVal) / (dx);
    float dpy_clause2 = (k/r * sin(a_tan) * dy) / d;
    float dpy = dpy_clause1 + dpy_clause2;
    
    // dpz
    float dpz = (k/r * sin(a_tan) * dz) / d;
    
    // dqx
    float dqx = dpx_clause1 - dpx_clause2;
    
    // dqy
    float dqy = -dpy_clause1 - dpy_clause2;
    
    // dqz
    float dqz = -dpz;
    
    Eigen::VectorXf retVal(6);
    retVal[0] = dpx;
    retVal[1] = dpy;
    retVal[2] = dpz;
    retVal[3] = dqx;
    retVal[4] = dqy;
    retVal[5] = dqz;
    return retVal;
}

Eigen::VectorXf Hessian::computeHessian_dfz(Eigen::Vector3f p, Eigen::Vector3f q, float r, float k) {
    float dx = abs(p[0] - q[0]);
    float dy = abs(p[1] - q[1]);
    float dz = abs(p[2] - q[2]);
    float dxy = sqrt(dx*dx + dy*dy);
    
    float d = sqrt(dx*dx + dy*dy + dz*dz);
    
    float acos2 = pow(acos(dz / dxy), 2);
    float asec2 = 1/acos2;
    
    float a_tan = atan(dz / dxy);
    float cos_atan = cos(a_tan);
    
    float longVal = (k/r * d) - k;
    
    float denom_cubed = pow(dxy, 3);
    
    // dpx
    float dpx_clause1 = (asec2 * cos_atan * dz * dx * longVal) / denom_cubed;
    float dpx_clause2 = (k/r * sin(a_tan) * dx) / d;
    float dpx = -dpx_clause1 + dpx_clause2;
    
    // dpy
    float dpy_clause1 = (asec2 * cos_atan * dz * dy * longVal) / denom_cubed;
    float dpy_clause2 = (k/r * sin(a_tan) * dy) / d;
    float dpy = -dpy_clause1 + dpy_clause2;
    
    // dpz
    float dpz_clause1 = (asec2 * cos_atan * longVal) / dxy;
    float dpz_clause2 = (k/r * sin(a_tan) * dz) / d;
    float dpz = dpz_clause1 + dpz_clause2;
    
    // dqx
    float dqx = dpx_clause1 - dpx_clause2;
    
    // dqy
    float dqy = dpy_clause1 - dpy_clause2;
    
    // dqz
    float dqz = -dpy_clause1 - dpy_clause2;
    
    Eigen::VectorXf retVal(6);
    retVal[0] = dpx;
    retVal[1] = dpy;
    retVal[2] = dpz;
    retVal[3] = dqx;
    retVal[4] = dqy;
    retVal[5] = dqz;
    return retVal;
}
