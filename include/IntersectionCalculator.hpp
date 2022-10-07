//
// Created by jy on 22. 10. 6.
//

#ifndef COLLISIONDETECTIONOBB_INTERSECTIONCALCULATOR_HPP
#define COLLISIONDETECTIONOBB_INTERSECTIONCALCULATOR_HPP

#include "Eigen/Eigen"
#include "iostream"

#include "SetObb.hpp"

struct box
{
    Eigen::VectorXd boxSize;
    Eigen::Vector3d boxCenter;
    Eigen::MatrixXd boxOrientation;
};

class IntersectionCalculator
{
public:
    bool intersectionChecker(box boxA, box boxB);

private:
    Eigen::MatrixXd valueCalculator(box boxA, box boxB);
    void calR0();
    void calR1();
    void calR();

private:

    Eigen::VectorXd mboxAextent;
    Eigen::VectorXd mboxBextent;

    Eigen::Vector3d mboxAcenter;
    Eigen::Vector3d mboxBcenter;

    Eigen::MatrixXd mboxAorientation;
    Eigen::MatrixXd mboxBorientation;

    Eigen::MatrixXd entriesMat = Eigen::MatrixXd(3,3);

    Eigen::VectorXd mR0 = Eigen::VectorXd(15);
    Eigen::VectorXd mR1 = Eigen::VectorXd(15);
    Eigen::VectorXd mR = Eigen::VectorXd(15);

    SetOBB setOBB;

};

#endif //COLLISIONDETECTIONOBB_INTERSECTIONCALCULATOR_HPP