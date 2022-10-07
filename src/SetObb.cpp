//
// Created by jy on 22. 10. 6.
//

#include "../include/SetObb.hpp"

Eigen::VectorXd SetOBB::SetBox(float length, float width, float height)
{
    Eigen::VectorXd boxSize = Eigen::VectorXd(3);
    boxSize(0) = length;
    boxSize(1) = width;
    boxSize(2) = height;

    return boxSize;
}

Eigen::MatrixXd SetOBB::SetOrientation(Eigen::VectorXd x_orientation, Eigen::VectorXd y_orientation, Eigen::VectorXd z_orientation)
{
    Eigen::MatrixXd boxOrientation = Eigen::MatrixXd(3, 3);

    boxOrientation.col(0) = x_orientation;
    boxOrientation.col(1) = y_orientation;
    boxOrientation.col(2) = z_orientation;

    return boxOrientation;
}