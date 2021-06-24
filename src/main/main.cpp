#include <iostream>

#include "kinematics_solver.h"

using namespace std;

int main()
{
    Position p = {0.168, -0.074, -0.311};
    JointConfiguration q = inverseKinematics(p, NUMERICAL);
    p = directKinematics(q);

    cout << p.x << " " << p.y << " " << p.z << endl;
    cout << q.q1 << " " << q.q2 << " " << q.q3 << endl;

    q = inverseKinematics(p, ANALYTICAL);
    p = directKinematics(q);
    cout << endl << p.x << " " << p.y << " " << p.z << endl;
    cout << q.q1 << " " << q.q2 << " " << q.q3 << endl;

    return 0;
}
