#include <iostream>
#include <math.h>

#include "inverse_kinematics.h"

using namespace std;

int main()
{
    Position p = {0.197, 0.084, -0.09};
    JointConfiguration q = inverseKinematics(p, {1, 1, 1});
    p = directKinematics(q);

    cout << p.x << " " << p.y << " " << p.z << endl;
    cout << q.q1 << " " << q.q2 << " " << q.q3 << endl;

    return 0;
}
