//
// Created by jy on 22. 10. 6.
//

#ifndef COLLISIONDETECTIONOBB_SETOBB_HPP
#define COLLISIONDETECTIONOBB_SETOBB_HPP

#include "Eigen/Eigen"

class SetOBB
{
public:

    Eigen::VectorXd SetBox(float length, float width, float height);
    Eigen::MatrixXd SetOrientation(Eigen::VectorXd x, Eigen::VectorXd y, Eigen::VectorXd z);

};


#endif //COLLISIONDETECTIONOBB_SETOBB_HPP
