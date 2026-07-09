#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "Eigen/Geometry"
using namespace Eigen;

class UKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    UKF();
    void initialize(const VectorXd &x0);
    VectorXd update(VectorXd &x, double dt, VectorXd accel, VectorXd gyro);
    Quaterniond getOrientation() const;

private:
    VectorXd x;  // State: [q0 q1 q2 q3 vx vy vz]
    MatrixXd P;  // Covariance
    MatrixXd Q;  // Process noise
    MatrixXd R;  // Measurement noise

    void normalizeQuaternion();
    MatrixXd perturbmat(const VectorXd &x, const MatrixXd &W);
    VectorXd perturbvec(VectorXd xapri,VectorXd update);
    VectorXd rotq(const VectorXd x, const VectorXd v);
    MatrixXd A(const MatrixXd X, double dt);
    VectorXd v2q(const VectorXd v);
    VectorXd q2v(const VectorXd q);
    VectorXd quatmult(const VectorXd Y, const VectorXd X);
    MatrixXd covar(const MatrixXd &data);
    VectorXd quatrotate(VectorXd q, VectorXd v);

};

#endif // UKF_H
