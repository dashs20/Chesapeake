#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"
#include "UKF/UKF.hpp"

class NAV {
public:
    NAV(GNCc cfg);
    NAVb update(const GNCb& gnc);

private:
    double dt_s;
    Eigen::VectorXd x;
    UKF ukf;
};