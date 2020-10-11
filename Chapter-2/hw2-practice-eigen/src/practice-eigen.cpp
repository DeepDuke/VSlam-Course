#include<iostream>

using namespace std;

#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

int main(int argc, char **argv){
    MatrixXd A = MatrixXd::Random(100, 100);
    A = A * A.transpose();
    Matrix<double, 100, 1> b = MatrixXd::Random(100, 1);
    Matrix<double, 100, 1> x;

    // QR decomposition
    clock_t t1 = clock();
    x = A.colPivHouseholderQr().solve(b);
    clock_t t2 = clock();
    cout << "\nTime of QR decomposition:\t"
         << 1000 * (t2 - t1) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "\nx = " << x.transpose() << endl;

    // Cholesky decomposition
    t1 = clock();
    x = A.ldlt().solve(b);
    t2 = clock();
    cout << "\nTime of Cholesky decomposition:\t"
         << 1000 * (t2 - t1) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "\nx = " << x.transpose() << endl;

    return 0;
}
