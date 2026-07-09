#include "VSM.hpp"

VSM::VSM(GNCc cfg) : cfg_data(cfg) {
    setup_vehicle_mode_machine();
    setup_attitude_mode_machine();
}

VSMb VSM::update(const GNCb& gnc) {
    VSMb vsmb;
    vsmb.state = vehicle_mode_machine.update(gnc.vsmb.state, gnc);
    vsmb.att_mode = attitude_mode_machine.update(gnc.vsmb.att_mode, gnc);
    return vsmb;
}

void VSM::setup_vehicle_mode_machine() {
    float arm_threshold_frac = cfg_data.vsmc.arm_threshold_frac;
    float rate_threshold_frac = cfg_data.vsmc.mode_rate_threshold_frac;

    State<STATE, GNCb> disarmed_state;
    disarmed_state.id = STATE::DISARMED;
    disarmed_state.exit_paths = {
        { STATE::RATE, [arm_threshold_frac, rate_threshold_frac](const GNCb& g) {
            return g.halb.rcrxb.arm_frac > arm_threshold_frac && g.halb.rcrxb.mode_frac < rate_threshold_frac;
        }},
        { STATE::ANGLE, [arm_threshold_frac, rate_threshold_frac](const GNCb& g) {
            return g.halb.rcrxb.arm_frac > arm_threshold_frac && g.halb.rcrxb.mode_frac >= rate_threshold_frac;
        }}
    };

    State<STATE, GNCb> rate_state;
    rate_state.id = STATE::RATE;
    rate_state.exit_paths = {
        { STATE::DISARMED, [arm_threshold_frac](const GNCb& g) {
            return g.halb.rcrxb.arm_frac <= arm_threshold_frac;
        }},
        { STATE::ANGLE, [rate_threshold_frac](const GNCb& g) {
            return g.halb.rcrxb.mode_frac >= rate_threshold_frac;
        }}
    };

    State<STATE, GNCb> angle_state;
    angle_state.id = STATE::ANGLE;
    angle_state.exit_paths = {
        { STATE::DISARMED, [arm_threshold_frac](const GNCb& g) {
            return g.halb.rcrxb.arm_frac <= arm_threshold_frac;
        }},
        { STATE::RATE, [rate_threshold_frac](const GNCb& g) {
            return g.halb.rcrxb.mode_frac < rate_threshold_frac;
        }}
    };

    vehicle_mode_machine.add_state(disarmed_state);
    vehicle_mode_machine.add_state(rate_state);
    vehicle_mode_machine.add_state(angle_state);
}

void VSM::setup_attitude_mode_machine() {
    State<ATT_MODE, GNCb> rate_state;
    rate_state.id = ATT_MODE::RATE;
    rate_state.exit_paths = {
        { ATT_MODE::ANGLE, [](const GNCb& g) {
            return g.vsmb.state == STATE::ANGLE;
        }}
    };

    State<ATT_MODE, GNCb> angle_state;
    angle_state.id = ATT_MODE::ANGLE;
    angle_state.exit_paths = {
        { ATT_MODE::RATE, [](const GNCb& g) {
            return g.vsmb.state == STATE::RATE || g.vsmb.state == STATE::DISARMED;
        }}
    };

    attitude_mode_machine.add_state(rate_state);
    attitude_mode_machine.add_state(angle_state);
}
