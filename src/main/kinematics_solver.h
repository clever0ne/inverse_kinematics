#pragma once

static const double l1 = 0.0838;
static const double l2 = 0.2;
static const double l3 = 0.2;

static const double q1_min = -0.8;
static const double q1_max = 0.8;
static const double q2_min = -1.04;
static const double q2_max = 4.18;
static const double q3_min = -2.68;
static const double q3_max = 0.09;

static const double epsilon = 0.0001;

enum SolverType
{
    NUMERICAL,
    ANALYTICAL
};

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

static const JointConfiguration initial_guess =
{
    (q1_min + q1_max) / 2,
    (q2_min + q2_max) / 2,
    (q3_min + q3_max) / 2
};

Jacobian calculateJacobian(const JointConfiguration jointConfiguration);
Position directKinematics(const JointConfiguration jointConfiguration);
JointConfiguration inverseKinematics(const Position position,
                                     const SolverType solverType = SolverType::NUMERICAL,
                                     const JointConfiguration initialGuess = initial_guess);
JointConfiguration numericalInverseKinematics(const Position position,
                                              const JointConfiguration initialGuess = initial_guess);
JointConfiguration analyticalInverseKinematics(const Position position,
                                               const JointConfiguration initialGuess = initial_guess);
