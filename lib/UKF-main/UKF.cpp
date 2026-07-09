#include "UKF.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;

UKF::UKF()
{
    P = MatrixXd::Identity(6,6) * 5e-2; // State Uncertainty
    Q = MatrixXd::Identity(6, 6) * 1e-2; // Process Noise
    R = MatrixXd::Identity(6, 6) * 5e-2; // Measurement Noise
    R(2, 2) = 8e-2;
    R.block(3, 3, 3, 3) = MatrixXd::Identity(3, 3) * .005;
}

void UKF::initialize(const VectorXd &x0)
{
    x = x0;                // initialize state vector
    normalizeQuaternion(); // normalize the quaternion part of the state vector   
}

void UKF::normalizeQuaternion()
{
    Vector4d q = x.head<4>();
    q.normalize();
    x.head<4>() = q;
}

VectorXd UKF::update(VectorXd &x, double dt, VectorXd accel, VectorXd gyro)
{
    // Sigma Points and Prediction
    MatrixXd S;
    S = P + Q * dt;
    LLT<MatrixXd> lltofS(S);
    S = lltofS.matrixL().transpose(); // seg fault
    MatrixXd W(6, 13);
    VectorXd zeroVector(6);
    zeroVector.setZero();
    W.block(0, 0, 6, 6) = sqrt(6) * S;
    W.block(0, 6, 6, 1) = zeroVector;
    W.block(0, 7, 6, 6) = -sqrt(6) * S;
    MatrixXd X = perturbmat(x, W);
    MatrixXd Y = A(X, dt);
    MatrixXd Ysvd = Y.block(0, 0, 4, 13) * Y.block(0, 0, 4, 13).transpose() / 13.0;
    JacobiSVD<MatrixXd> svd(Ysvd, ComputeThinU | ComputeThinV);
    MatrixXd Y_q(4, 4);
    Y_q = svd.matrixV();
    MatrixXd Y_qU(4, 4);
    Y_qU = svd.matrixU();
    VectorXd xapri(7);
    xapri.head<4>() = Y_q.col(0) / (Y_q.col(0).norm());
    if ((Y_q.col(0).norm()) <= 1e-10)
    {
        xapri(0) = 1;
        xapri(1) = 0;
        xapri(2) = 0;
        xapri(3) = 0;
    }
    xapri(4) = X.row(4).mean();
    xapri(5) = X.row(5).mean();
    xapri(6) = X.row(6).mean();
    MatrixXd e_q(4, 13);
    VectorXd e_qvec(4);
    e_qvec << 1, -1, -1, -1;
    for (int i = 0; i < Y.cols(); i++)
    {
        e_q.col(i) = quatmult(Y.col(i).head<4>(), xapri.head<4>().array() * e_qvec.array());
    }
    MatrixXd Wp(6, 13);
    for (int i = 0; i < Y.cols(); i++)
    {
        Wp.col(i).head<3>() = q2v(e_q.col(i));
        Wp.col(i).tail<3>() = Y.col(i).tail<3>() - xapri.tail<3>();
    }
    MatrixXd Papri(6, 6);
    Papri = covar(Wp.transpose());

    // Measurement and Innovation
    MatrixXd Z(6, 13);
    Vector3d gravec(0, 0, 1);
    for (int i = 0; i < Y.cols(); i++)
    {
        Z.col(i).head<3>() = quatrotate(Y.col(i).head<4>(), gravec);
        Z.col(i).tail<3>() = Y.col(i).tail<3>();
    }
    VectorXd zapri(6); // 1x6 dim
    zapri(0) = Z.row(0).mean();
    zapri(1) = Z.row(1).mean();
    zapri(2) = Z.row(2).mean();
    zapri(3) = Z.row(3).mean();
    zapri(4) = Z.row(4).mean();
    zapri(5) = Z.row(5).mean();
    MatrixXd Pzz(6, 6);
    Pzz = covar(Z.transpose());
    if (accel.norm() == 0)
    {
        accel = accel;
    }
    else
    {
        accel = accel / accel.norm();
    }
    zapri.head<3>() = zapri.head<3>() / (zapri.head<3>().norm());
    VectorXd v(6);
    v.head<3>() = accel - zapri.head<3>();
    v.tail<3>() = gyro - zapri.tail<3>();
    MatrixXd Pvv(6, 6);
    Pvv = Pzz + R;

    // Kalman Update
    MatrixXd Pxz(6, 6);
    MatrixXd Zerr(6, 13);
    for (int i = 0; i < Z.cols(); i++)
    {
        Zerr.col(i) = Z.col(i) - zapri;
    }
    Pxz = Wp * Zerr.transpose() / 13.0;
    MatrixXd K(6, 6);
    K = Pxz * (Pvv.inverse());
    VectorXd stateupdate(6);
    VectorXd flipvector(6);
    flipvector << -1, //not sure why needed, this is due to the e_q flip.
        -1,
        -1,
        1,
        1,
        1;
    stateupdate = (K * v).array() * flipvector.array();
    x = perturbvec(xapri, stateupdate);
    this->x = x;
    P = Papri - K * Pvv * K.transpose();
    return x;
}

MatrixXd UKF::perturbmat(const VectorXd &x, const MatrixXd &W)
{
    MatrixXd X(7, W.cols());
    for (int i = 0; i < W.cols(); i++)
    {
        X.col(i).head<4>() = rotq(x.head<4>(), W.col(i).head<3>());
        X.col(i).tail<3>() = x.tail<3>() + W.col(i).tail<3>();
    }
    return X;
}
VectorXd UKF::perturbvec(VectorXd xapri, VectorXd update)
{
    VectorXd x(7);
    x.head<4>() = rotq(xapri.head<4>(), update.head<3>());
    x.tail<3>() = xapri.tail<3>() + update.tail<3>();
    return x;
}

VectorXd UKF::rotq(const VectorXd x, const VectorXd v)
{
    VectorXd qvec(4);
    qvec = quatmult(v2q(v), x);
    qvec.normalize();
    return qvec;
}

MatrixXd UKF::A(const MatrixXd X, double dt)
{
    MatrixXd Y(7, 13);
    for (int i = 0; i < X.cols(); i++)
    {
        Y.col(i).head<4>() = rotq(X.col(i).head<4>(), (X.col(i).tail<3>() * dt));
        Y.col(i).tail<3>() = X.col(i).tail<3>();
    }
    return Y;
}

VectorXd UKF::v2q(const VectorXd v)
{
    float vnorm = v.norm();
    VectorXd qvec(4);
    qvec(0) = cos(vnorm * 0.5);
    qvec.tail<3>() = v * sin(vnorm * 0.5) / (vnorm);
    if (vnorm <= 1e-6)
    {
        qvec(0) = 1;
        qvec.tail<3>() << 0,
            0,
            0;
    }
    qvec.normalize();
    return qvec;
}

VectorXd UKF::q2v(const VectorXd q)
{
    float angle = 2 * acos(q(0));
    VectorXd v(3);
    v = q.tail<3>().array() * angle / (sin(angle / 2));
    if (angle <= 1e-6)
    {
        v << 0,
            0,
            0;
    }
    return v;
}

VectorXd UKF::quatmult(const VectorXd Y, const VectorXd X)
{
    VectorXd qvec(4);
    qvec(0) = Y(0) * X(0) - Y(1) * X(1) - Y(2) * X(2) - Y(3) * X(3);
    qvec(1) = Y(1) * X(0) + Y(0) * X(1) - Y(3) * X(2) + Y(2) * X(3);
    qvec(2) = Y(2) * X(0) + Y(3) * X(1) + Y(0) * X(2) - Y(1) * X(3);
    qvec(3) = Y(3) * X(0) - Y(2) * X(1) + Y(1) * X(2) + Y(0) * X(3);
    qvec.normalize();
    return qvec;
}

MatrixXd UKF::covar(const MatrixXd &data) // check if correct
{
    MatrixXd centeredData = data.rowwise() - data.colwise().mean();
    MatrixXd cov = (centeredData.adjoint() * centeredData) / (data.rows() - 1.0);
    return cov;
}

VectorXd UKF::quatrotate(VectorXd q, VectorXd v)
{
    MatrixXd quatmat(3, 3);
    quatmat << (1 - 2 * q(2) * q(2) - 2 * q(3) * q(3)), 2 * (q(1) * q(2) + q(0) * q(3)), 2 * (q(1) * q(3) - q(0) * q(2)),
        2 * (q(1) * q(2) - q(0) * q(3)), (1 - 2 * q(1) * q(1) - 2 * q(3) * q(3)), 2 * (q(2) * q(3) + q(0) * q(1)),
        2 * (q(1) * q(3) + q(0) * q(2)), 2 * (q(2) * q(3) - q(0) * q(1)), (1 - 2 * q(1) * q(1) - 2 * q(2) * q(2));
    VectorXd newvec(3);
    newvec = quatmat * v;
    return newvec;
}

Quaterniond UKF::getOrientation() const
{
    return Quaterniond(x(0), x(1), x(2), x(3));
}
