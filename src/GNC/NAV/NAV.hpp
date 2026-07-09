#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"
#include "UKF.hpp"

class NAV{
    public:
        UKF ukf;
        Eigen::VectorXd x;
        double dt_s;

        NAV(GNCc cfg);
        GNCb update(GNCb gnc);
};