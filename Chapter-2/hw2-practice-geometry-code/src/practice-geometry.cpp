#include<iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main(int argc, char **argv){
    // World to robot 1
    Isometry3d  T1w = Isometry3d::Identity();
    Quaterniond q1 = {0.55, 0.3, 0.2, 0.2};
    T1w.rotate(q1.normalized());
    T1w.pretranslate(Vector3d(0.7, 1.1, 0.2));

    // World to robot 2
    Isometry3d  T2w = Isometry3d::Identity();
    Quaterniond q2 = {-0.1, 0.3, -0.7, 0.2};
    T2w.rotate(q2.normalized());
    T2w.pretranslate(Vector3d(-0.1, 0.4, 0.8));

    Vector3d p1(0.5, -0.1, 0.2);

    Vector3d p2 = T2w * T1w.inverse() * p1;

    cout.precision(6);
    cout << "p2 = " << p2.transpose() << endl;
    return 0;
}