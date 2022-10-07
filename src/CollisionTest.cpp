//
// Created by jy on 22. 10. 6.
//

#include "../include/CollisionTest.hpp"

int main()
{
    bool testSetA = true;
    bool testSetB = true;

    bool resultSetA = false;
    bool resultSetB = false;

    Eigen::VectorXd boxA_x = Eigen::VectorXd(3);
    Eigen::VectorXd boxA_y = Eigen::VectorXd(3);
    Eigen::VectorXd boxA_z = Eigen::VectorXd(3);

    Eigen::VectorXd boxB_x = Eigen::VectorXd(3);
    Eigen::VectorXd boxB_y = Eigen::VectorXd(3);
    Eigen::VectorXd boxB_z = Eigen::VectorXd(3);

    Eigen::VectorXd boxC_x = Eigen::VectorXd(3);
    Eigen::VectorXd boxC_y = Eigen::VectorXd(3);
    Eigen::VectorXd boxC_z = Eigen::VectorXd(3);

    box boxA;
    box boxB;
    box boxC;

    boxA_x << 1,0,0;
    boxA_y << 0,1,0;
    boxA_z << 0,0,1;

    boxB_x << 1,0,0;
    boxB_y << 0,1,0;
    boxB_z << 0,0,1;

    boxC_x << 1,0,0;
    boxC_y << 0,1,0;
    boxC_z << 0,0,1;

    boxA.boxSize = setObb.SetBox(5,5,5);
    boxB.boxSize = setObb.SetBox(5,5,5);
    boxC.boxSize = setObb.SetBox(5,5,5);

    boxA.boxOrientation = setObb.SetOrientation(boxA_x,boxA_y,boxA_z);
    boxB.boxOrientation = setObb.SetOrientation(boxB_x,boxB_y,boxB_z);
    boxC.boxOrientation = setObb.SetOrientation(boxC_x,boxC_y,boxC_z);

    boxA.boxCenter = setObb.SetBox(2.5,2.5,2.5);
    boxB.boxCenter = setObb.SetBox(10,10,10);
    boxC.boxCenter = setObb.SetBox(10,10,0);

    if(testSetA)
    {
        resultSetA = intersectionCal.intersectionChecker(boxA,boxB);
        if (resultSetA)
        {
            std::cout << "\ntest set A occur collision" << std::endl;
        }
        else
        {
            std::cout << "\ntest set A didn't occur collision" << std::endl;
        }

    }
    if(testSetB)
    {
        resultSetB = intersectionCal.intersectionChecker(boxB,boxC);
        if (resultSetB)
        {
            std::cout << "\ntest set B occur collision" << std::endl;
        }
        else
        {
            std::cout << "\ntest set B didn't occur collision" << std::endl;
        }

    }
}