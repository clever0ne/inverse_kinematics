#include "inverse_kinematics.h"

#include <iostream>
#include <math.h>

using namespace std;

const double l1 = 0.0832;
const double l2 = 0.2;
const double l3 = 0.2;

const double epsilon = 0.0001;

Position directKinematics(const JointConfiguration jointConfiguration)
{
    double q1 = jointConfiguration.q1;
    double q2 = jointConfiguration.q2;
    double q3 = jointConfiguration.q3;

    // Уравнения прямой кинематики:
    // F(q) = p (1)
    // l2 * sin(q2) + l3 * sin(q2 + q3) = x
    // l1 * cos(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1) = y
    // l1 * sin(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1) = z

    double x = l2 * sin(q2) + l3 * sin(q2 + q3);
    double y = l1 * cos(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1);
    double z = l1 * sin(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1);

    Position position = {x, y, z};
    return position;
}

Jacobian calculateJacobian(const JointConfiguration jointConfiguration)
{
    double q1 = jointConfiguration.q1;
    double q2 = jointConfiguration.q2;
    double q3 = jointConfiguration.q3;

    // Якобиан системы уравнений (1):
    // J(q) = dF(q)/dq (2)

    // df1/dq:
    // 0
    // l2 * cos(q2) + l3 * cos(q2 + q3)
    // l3 * cos(q2 + q3)

    // df2/dq:
    // -l1 * sin(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1)
    // (l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1)
    // l3 * sin(q2 + q3) * sin(q1)

    // df3/dq:
    // l1 * cos(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1)
    // -(l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1)
    // -l3 * sin(q2 + q3) * cos(q1)

    Jacobian jacobian =
    {
        {
            0,
            l2 * cos(q2) + l3 * cos(q2 + q3),
            l3 * cos(q2 + q3),
            -l1 * sin(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1),
            (l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1),
            l3 * sin(q2 + q3) * sin(q1),
            l1 * cos(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1),
            -(l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1),
            -l3 * sin(q2 + q3) * cos(q1)
        }
    };

    return jacobian;
}

Jacobian calculateInverseJacobian(const Jacobian jacobian)
{
    double determinant = jacobian.data[0] * (jacobian.data[4] * jacobian.data[8] -
                                             jacobian.data[7] * jacobian.data[5]) -
                         jacobian.data[1] * (jacobian.data[3] * jacobian.data[8] -
                                             jacobian.data[6] * jacobian.data[5]) +
                         jacobian.data[2] * (jacobian.data[3] * jacobian.data[7] -
                                             jacobian.data[6] * jacobian.data[4]);

    Jacobian inverseJacobian
    {
        {
            (jacobian.data[4] * jacobian.data[8] -
             jacobian.data[5] * jacobian.data[7]) / determinant,
            (jacobian.data[2] * jacobian.data[7] -
             jacobian.data[1] * jacobian.data[8]) / determinant,
            (jacobian.data[1] * jacobian.data[5] -
             jacobian.data[2] * jacobian.data[4]) / determinant,
            (jacobian.data[5] * jacobian.data[6] -
             jacobian.data[3] * jacobian.data[8]) / determinant,
            (jacobian.data[0] * jacobian.data[8] -
             jacobian.data[2] * jacobian.data[6]) / determinant,
            (jacobian.data[2] * jacobian.data[3] -
             jacobian.data[0] * jacobian.data[5]) / determinant,
            (jacobian.data[3] * jacobian.data[7] -
             jacobian.data[4] * jacobian.data[6]) / determinant,
            (jacobian.data[1] * jacobian.data[6] -
             jacobian.data[0] * jacobian.data[7]) / determinant,
            (jacobian.data[0] * jacobian.data[4] -
             jacobian.data[1] * jacobian.data[3]) / determinant
        }
    };

    return inverseJacobian;
}

JointConfiguration calculateNextJointConfiguration(const Position position,
                                                   const Position currentPosition,
                                                   const JointConfiguration currentJointConfiguration)
{
    Jacobian jacobian = calculateJacobian(currentJointConfiguration);
    Jacobian inverseJacobian = calculateInverseJacobian(jacobian);

    double q1 = currentJointConfiguration.q1 +
                inverseJacobian.data[0] * (position.x - currentPosition.x) +
                inverseJacobian.data[1] * (position.y - currentPosition.y) +
                inverseJacobian.data[2] * (position.z - currentPosition.z);
    double q2 = currentJointConfiguration.q2 +
                inverseJacobian.data[3] * (position.x - currentPosition.x) +
                inverseJacobian.data[4] * (position.y - currentPosition.y) +
                inverseJacobian.data[5] * (position.z - currentPosition.z);
    double q3 = currentJointConfiguration.q3 +
                inverseJacobian.data[6] * (position.x - currentPosition.x) +
                inverseJacobian.data[7] * (position.y - currentPosition.y) +
                inverseJacobian.data[8] * (position.z - currentPosition.z);

    JointConfiguration nextJointConfiguration = {q1, q2, q3};
    return nextJointConfiguration;
}

JointConfiguration inverseKinematics(const Position position, const JointConfiguration initialGuess)
{
    double q1 = initialGuess.q1;
    double q2 = initialGuess.q2;
    double q3 = initialGuess.q3;

    bool isSolutionFound = false;
    while (isSolutionFound != true)
    {
        JointConfiguration currentJointConfiguration = {q1, q2, q3};
        Position currentPosition = directKinematics(currentJointConfiguration);

        JointConfiguration nextJointConfiguration =
                calculateNextJointConfiguration(position, currentPosition, currentJointConfiguration);
        Position nextPosition = directKinematics(nextJointConfiguration);

        q1 = nextJointConfiguration.q1;
        q2 = nextJointConfiguration.q2;
        q3 = nextJointConfiguration.q3;

        if (abs(nextPosition.x - currentPosition.x) < epsilon &&
            abs(nextPosition.y - currentPosition.y) < epsilon &&
            abs(nextPosition.z - currentPosition.z) < epsilon)
        {
            isSolutionFound = true;
        }
    }

    JointConfiguration jointConfiguration =
    {
        q1 - static_cast<double>(round(q1 / (2 * M_PI)) * (2 * M_PI)),
        q2 - static_cast<double>(round(q2 / (2 * M_PI)) * (2 * M_PI)),
        q3 - static_cast<double>(round(q3 / (2 * M_PI)) * (2 * M_PI))
    };
    return jointConfiguration;
}
