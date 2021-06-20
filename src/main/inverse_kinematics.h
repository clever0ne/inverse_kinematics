#pragma once

typedef struct Position
{
    double x;
    double y;
    double z;
} Position;

typedef struct JointConfiguration
{
    double q1;
    double q2;
    double q3;
} JointConfiguration;

typedef struct Jacobian
{
    double data[9];
} Jacobian;

Position directKinematics(const JointConfiguration jointConfiguration);
JointConfiguration inverseKinematics(const Position position, const JointConfiguration initialGuess = {0, 0, 0});
