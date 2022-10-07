//
// Created by jy on 22. 10. 6.
//
#include "../include/IntersectionCalculator.hpp"
#include "cmath"

Eigen::MatrixXd IntersectionCalculator::valueCalculator(box boxA, box boxB)
{
    mboxAextent = 0.5 * boxA.boxSize;
    mboxBextent = 0.5 * boxB.boxSize;

    mboxAcenter = boxA.boxCenter;
    mboxBcenter = boxB.boxCenter;

    mboxAorientation = boxA.boxOrientation;
    mboxBorientation = boxB.boxOrientation;

    entriesMat = mboxAorientation.transpose() * mboxBorientation;

    calR0();
    calR1();
    calR();

    Eigen::MatrixXd rValueMatrix = Eigen::MatrixXd(15,3);
    rValueMatrix.col(0) = mR0;
    rValueMatrix.col(1) = mR1;
    rValueMatrix.col(2) = mR;

//    std::cout << rValueMatrix << std::endl;

    return rValueMatrix;
}

/**
 * if obb-obb collision occur function stop immediately and return true
 * @param boxA
 * @param boxB
 * @return
 */
bool IntersectionCalculator::intersectionChecker(box boxA, box boxB)
{
    Eigen::MatrixXd rValueMatrix = Eigen::MatrixXd(15,3);
    rValueMatrix = valueCalculator(boxA,boxB);

    for (int idx=0 ; idx < 15 ; idx++)
    {
        if(rValueMatrix(idx,2) + 0.0001 > rValueMatrix(idx,0) + rValueMatrix(idx,1))
        {
            continue;
        }
        else
        {
            return true;
        }
    }
    return false;
}

void IntersectionCalculator::calR0()
{
    mR0(0)  = mboxAextent(0);
    mR0(1)  = mboxAextent(1);
    mR0(2)  = mboxAextent(2);

    mR0(3)  = mboxAextent(0) * abs(entriesMat(0, 0)) + mboxAextent(1) * abs(entriesMat(1, 0)) + mboxAextent(2) * abs(entriesMat(2, 0));
    mR0(4)  = mboxAextent(0) * abs(entriesMat(0, 1)) + mboxAextent(1) * abs(entriesMat(1, 1)) + mboxAextent(2) * abs(entriesMat(2, 1));
    mR0(5)  = mboxAextent(0) * abs(entriesMat(0, 2)) + mboxAextent(1) * abs(entriesMat(1, 2)) + mboxAextent(2) * abs(entriesMat(2, 2));

    mR0(6)  = mboxAextent(1) * abs(entriesMat(2, 0)) + mboxAextent(2) * abs(entriesMat(1, 0));
    mR0(7)  = mboxAextent(1) * abs(entriesMat(2, 1)) + mboxAextent(2) * abs(entriesMat(1, 1));
    mR0(8)  = mboxAextent(1) * abs(entriesMat(2, 2)) + mboxAextent(2) * abs(entriesMat(1, 2));

    mR0(9)  = mboxAextent(0) * abs(entriesMat(2, 0)) + mboxAextent(2) * abs(entriesMat(0, 0));
    mR0(10) = mboxAextent(0) * abs(entriesMat(2, 1)) + mboxAextent(2) * abs(entriesMat(0, 1));
    mR0(11) = mboxAextent(0) * abs(entriesMat(2, 2)) + mboxAextent(2) * abs(entriesMat(0, 2));

    mR0(12) = mboxAextent(0) * abs(entriesMat(1, 0)) + mboxAextent(1) * abs(entriesMat(0, 0));
    mR0(13) = mboxAextent(0) * abs(entriesMat(1, 1)) + mboxAextent(1) * abs(entriesMat(0, 1));
    mR0(14) = mboxAextent(0) * abs(entriesMat(1, 2)) + mboxAextent(1) * abs(entriesMat(0, 2));
}


void IntersectionCalculator::calR1()
{
    mR1(0)  = mboxBextent(0) * abs(entriesMat(0, 0)) + mboxBextent(1) * abs(entriesMat(0, 1)) + mboxBextent(2) * abs(entriesMat(0, 2));
    mR1(1)  = mboxBextent(0) * abs(entriesMat(1, 0)) + mboxBextent(1) * abs(entriesMat(1, 1)) + mboxBextent(2) * abs(entriesMat(1, 2));
    mR1(2)  = mboxBextent(0) * abs(entriesMat(2, 0)) + mboxBextent(1) * abs(entriesMat(2, 1)) + mboxBextent(2) * abs(entriesMat(2, 2));

    mR1(3)  = mboxBextent(0);
    mR1(4)  = mboxBextent(1);
    mR1(5)  = mboxBextent(2);

    mR1(6)  = mboxBextent(1) * abs(entriesMat(0, 2)) + mboxBextent(2) * abs(entriesMat(0, 1));
    mR1(7)  = mboxBextent(0) * abs(entriesMat(0, 2)) + mboxBextent(2) * abs(entriesMat(0, 0));
    mR1(8)  = mboxBextent(0) * abs(entriesMat(0, 1)) + mboxBextent(1) * abs(entriesMat(0, 0));

    mR1(9)  = mboxBextent(1) * abs(entriesMat(1, 2)) + mboxBextent(2) * abs(entriesMat(1, 1));
    mR1(10) = mboxBextent(0) * abs(entriesMat(1, 2)) + mboxBextent(2) * abs(entriesMat(1, 0));
    mR1(11) = mboxBextent(0) * abs(entriesMat(1, 1)) + mboxBextent(1) * abs(entriesMat(1, 0));

    mR1(12) = mboxBextent(1) * abs(entriesMat(2, 2)) + mboxBextent(2) * abs(entriesMat(2, 1));
    mR1(13) = mboxBextent(0) * abs(entriesMat(2, 2)) + mboxBextent(2) * abs(entriesMat(2, 0));
    mR1(14) = mboxBextent(0) * abs(entriesMat(2, 1)) + mboxBextent(1) * abs(entriesMat(2, 0));
}

void IntersectionCalculator::calR()
{
    Eigen::VectorXd centerSub = mboxBcenter - mboxAcenter;

    Eigen::VectorXd axesA0 = mboxAorientation.col(0);
    Eigen::VectorXd axesA1 = mboxAorientation.col(1);
    Eigen::VectorXd axesA2 = mboxAorientation.col(2);

    Eigen::VectorXd axesB0 = mboxBorientation.col(0);
    Eigen::VectorXd axesB1 = mboxBorientation.col(1);
    Eigen::VectorXd axesB2 = mboxBorientation.col(2);

    mR(0)  = abs(axesA0.dot(centerSub));
    mR(1)  = abs(axesA1.dot(centerSub));
    mR(2)  = abs(axesA2.dot(centerSub));

    mR(3)  = abs(axesB0.dot(centerSub));
    mR(4)  = abs(axesB1.dot(centerSub));
    mR(5)  = abs(axesB2.dot(centerSub));

    mR(6)  = abs(entriesMat(1,0) * axesA2.dot(centerSub) - entriesMat(2,0) * axesA1.dot(centerSub));
    mR(7)  = abs(entriesMat(1,1) * axesA2.dot(centerSub) - entriesMat(2,1) * axesA1.dot(centerSub));
    mR(8)  = abs(entriesMat(1,2) * axesA2.dot(centerSub) - entriesMat(2,2) * axesA1.dot(centerSub));

    mR(9)  = abs(entriesMat(2,0) * axesA0.dot(centerSub) - entriesMat(0,0) * axesA2.dot(centerSub));
    mR(10)  = abs(entriesMat(2,1) * axesA0.dot(centerSub) - entriesMat(0,1) * axesA2.dot(centerSub));
    mR(11)  = abs(entriesMat(2,2) * axesA0.dot(centerSub) - entriesMat(0,2) * axesA2.dot(centerSub));

    mR(12)  = abs(entriesMat(0,0) * axesA1.dot(centerSub) - entriesMat(1,0) * axesA0.dot(centerSub));
    mR(13)  = abs(entriesMat(0,1) * axesA1.dot(centerSub) - entriesMat(1,1) * axesA0.dot(centerSub));
    mR(14)  = abs(entriesMat(0,2) * axesA1.dot(centerSub) - entriesMat(1,2) * axesA0.dot(centerSub));
}
