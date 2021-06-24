#include <math.h>
#include <limits>

#include "kinematics_solver.h"

using namespace std;

static bool isJointConfigurationValid(const JointConfiguration jointConfiguration)
{
    if (jointConfiguration.q1 < q1_min || jointConfiguration.q1 > q1_max)
    {
        return false;
    }

    if (jointConfiguration.q2 < q2_min || jointConfiguration.q2 > q2_max)
    {
        return false;
    }

    if (jointConfiguration.q3 < q3_min || jointConfiguration.q3 > q3_max)
    {
        return false;
    }

    return true;
}

Position directKinematics(const JointConfiguration jointConfiguration)
{
    double q1 = jointConfiguration.q1;
    double q2 = jointConfiguration.q2;
    double q3 = jointConfiguration.q3;

    // Уравнения прямой кинематики:
    // F(q) = p (1)
    // -l2 * sin(q2) - l3 * sin(q2 + q3) = x
    // l1 * cos(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1) = y
    // l1 * sin(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1) = z

    double x = -l2 * sin(q2) - l3 * sin(q2 + q3);
    double y = l1 * cos(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1);
    double z = l1 * sin(q1) - (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1);

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
    // -l2 * cos(q2) - l3 * cos(q2 + q3)
    // -l3 * cos(q2 + q3)

    // df2/dq:
    // -l1 * sin(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1)
    // -(l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1)
    // -l3 * sin(q2 + q3) * sin(q1)

    // df3/dq:
    // l1 * cos(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1)
    // (l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1)
    // l3 * sin(q2 + q3) * cos(q1)

    Jacobian jacobian =
    {
        {
            0,
            -l2 * cos(q2) - l3 * cos(q2 + q3),
            -l3 * cos(q2 + q3),
            -l1 * sin(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * cos(q1),
            -(l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1),
            -l3 * sin(q2 + q3) * sin(q1),
            l1 * cos(q1) + (l2 * cos(q2) + l3 * cos(q2 + q3)) * sin(q1),
            (l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1),
            l3 * sin(q2 + q3) * cos(q1)
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

JointConfiguration numericalInverseKinematics(const Position position,
                                              const JointConfiguration initialGuess)
{
    double q1 = initialGuess.q1;
    double q2 = initialGuess.q2;
    double q3 = initialGuess.q3;

    bool isSolutionFound = false;
    while (isSolutionFound != true)
    {
        JointConfiguration currentJointConfiguration = {q1, q2, q3};
        Position currentPosition = directKinematics(currentJointConfiguration);

        JointConfiguration nextJointConfiguration = calculateNextJointConfiguration(position,
                                                                                    currentPosition,
                                                                                    currentJointConfiguration);
        Position nextPosition = directKinematics(nextJointConfiguration);

        q1 = nextJointConfiguration.q1;
        q2 = nextJointConfiguration.q2;
        q3 = nextJointConfiguration.q3;

        if (hypot(nextPosition.x - currentPosition.x,
                  nextPosition.y - currentPosition.y,
                  nextPosition.z - currentPosition.z) < epsilon)
        {
            isSolutionFound = true;
        }
    }

    JointConfiguration jointConfiguration =
    {
        q1 - round(q1 / (2 * M_PI)) * (2 * M_PI),
        q2 - round(q2 / (2 * M_PI)) * (2 * M_PI),
        q3 - round(q3 / (2 * M_PI)) * (2 * M_PI)
    };

    return jointConfiguration;
}

JointConfiguration analyticalInverseKinematics(const Position position,
                                               const JointConfiguration initialGuess)
{
    JointConfiguration jointConfigurations[4];

    double sumOfSquares = position.y * position.y + position.z * position.z;
    double squareRoot = sqrt(sumOfSquares - l1 * l1);

    for (int i = 0; i < 2; i++)
    {
        double cosq1 = (position.y * l1 + (i == 0 ? 1 : -1) * position.z * squareRoot) / sumOfSquares;
        double sinq1 = (position.z * l1 + (i == 0 ? -1 : 1) * position.y * squareRoot) / sumOfSquares;
        double q1 = atan2(sinq1, cosq1);

        double x = 0;
        double y = l1 * cos(q1);
        double z = l1 * sin(q1);

        double l = hypot(position.x - x, position.y - y, position.z - z);
        int n = (l > l1 * M_SQRT2 || l > l1 * M_SQRT2) ? -1 : 1;

        double cosq3 = (l2 * l2 + l3 * l3 - l * l) / (2 * l2 * l3);

        for (int j = 0; j < 2; j++)
        {
            double sinq3 = sqrt(1 - cosq3 * cosq3);
            double q3 = atan2((j == 0 ? 1 : -1) * sinq3, n * cosq3);

            double a1 = l2 + l3 * cos(q3);
            double a2 = l3 * sin(q3);
            double detA = a1 * a1 + a2 * a2;

            double b1 = -position.x;
            double b2 = position.y * sin(q1) - position.z * cos(q1);

            double sinq2 = (b1 * a1 - b2 * a2) / detA;
            double cosq2 = (a1 * b2 + a2 * b1) / detA;
            double q2 = atan2(sinq2, cosq2);

            jointConfigurations[i * 2 + j] = {q1, q2, q3};
        }
    }

    JointConfiguration jointConfiguration;
    double minError = numeric_limits<double>::max();

    for (int k = 0; k < 4; k++)
    {
        if (isJointConfigurationValid(jointConfigurations[k]) != true)
        {
            continue;
        }

        double error = hypot(jointConfigurations[k].q1 - initialGuess.q1,
                             jointConfigurations[k].q2 - initialGuess.q2,
                             jointConfigurations[k].q3 - initialGuess.q3);

        if (error < minError)
        {
            minError = error;
            jointConfiguration = jointConfigurations[k];
        }
    }

    return jointConfiguration;
}

JointConfiguration inverseKinematics(const Position position,
                                     const SolverType solverType,
                                     const JointConfiguration initialGuess)
{
    switch (solverType)
    {
        case SolverType::NUMERICAL:
        {
            return numericalInverseKinematics(position, initialGuess);
        }
        case SolverType::ANALYTICAL:
        {
            return analyticalInverseKinematics(position, initialGuess);
        }
    }

    return numericalInverseKinematics(position, initialGuess);
}
