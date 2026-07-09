# UKF
Unscented Kalman Filter for attitude estimation using accelerometer and gyroscope data. The Unscented Kalman Filter (UKF) is a state estimation algorithm designed for nonlinear systems. It builds upon the standard Kalman filter but uses the unscented transform to better handle the non-linearities inherent in many real-world systems. Instead of linearizing the system equations (as in the Extended Kalman Filter), the UKF employs a deterministic sampling approach called the unscented transform to approximate the probability distribution of the state.

## Dependencies

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## Installation

To use this library in a CMake build, you only need Eigen included in CMake on your RaspberryPi. (or in same folder location as UKF.hpp)

## Usage
Add UKF object 
```
UKF ukf;
```

Initialize UKF
```
x << 1, //starting vector (quaternion [1-4], gyro [5-7])
0,
0,
0,
0,
0,
0;

ukf.initialize(x);
```

Update UKF and poll values
```
x = ukf.update(x, .001, accel, gyro); // accel and gyro are Eigen::Vector3d and dt is double(time between last update and                                          //this update, can be variable). x is the state vector (7) that has already been init.
            qw = x(0);
            qx = x(1);
            qy = x(2);
            qz = x(3);
            gyrox = x(4);
            gyroy = x(5);
            gyroz = x(6);
```
Tune parameters of variations and noise IN UKF.cpp
```
UKF::UKF()
{
    P = MatrixXd::Identity(6,6) * 5e-2; // State Uncertainty
    Q = MatrixXd::Identity(6, 6) * 1e-2; // Process Noise
    R = MatrixXd::Identity(6, 6) * 5e-2; // Measurement Noise
    R(2, 2) = 8e-2;
    R.block(3, 3, 3, 3) = MatrixXd::Identity(3, 3) * .005;
}
```

## Notes
- Make sure you're not using the same poll from the IMU as this would be "doubling up" on the sensor reading and not using the correct distribution of the data
- Sample the UKF at the same rate as the IMU, ensuring the data is fresh and you have a correct dt to use in the UKF.
- In the incl folder, there are versions of UKF for MATLAB, which this is based on, from Prof. Justin Yim's Master's coursework at UPenn.
## Resources
- https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
- [E. Kraft, "A quaternion-based unscented Kalman filter for orientation tracking," Sixth International Conference of Information Fusion, 2003. Proceedings of the, Cairns, QLD, Australia, 2003, pp. 47-54, doi: 10.1109/ICIF.2003.177425.](https://ieeexplore.ieee.org/document/1257247)
## Awards
- This material is based upon work supported by NASA under award No. 80NSSC25K7609
