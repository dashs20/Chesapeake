#include "RPI.hpp"
#include "../packager.hpp"

RPI::RPI(RPIc cfg) : rpic(cfg), port(nullptr), loop_count(0) {
    if (rpic.enabled) {
        if (rpic.uart_id == 1) {
            Serial1.setTX(rpic.tx_pin);
            Serial1.setRX(rpic.rx_pin);
            Serial1.begin(rpic.baudrate);
            port = &Serial1;
        } else if (rpic.uart_id == 2) {
            Serial2.setTX(rpic.tx_pin);
            Serial2.setRX(rpic.rx_pin);
            Serial2.begin(rpic.baudrate);
            port = &Serial2;
        }
    }
}

RPI::~RPI() {}

void RPI::update(const ALLb& allb_km1) {
    if (!rpic.enabled || port == nullptr) {
        return;
    }

    loop_count++;
    if (loop_count < rpic.rate_divisor) {
        return;
    }
    loop_count = 0;

    uint8_t buffer[350];
    size_t len = Packager::package_allb(allb_km1, buffer, sizeof(buffer));
    if (len > 0) {
        port->write(buffer, len);
    }
}
