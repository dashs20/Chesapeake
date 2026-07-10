#include "RPI.hpp"

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

    uint8_t header[4];
    header[0] = 0xAA;
    header[1] = 0xBB;
    header[2] = 0x01;
    header[3] = sizeof(ALLb);

    const uint8_t* payload = reinterpret_cast<const uint8_t*>(&allb_km1);
    uint16_t checksum = calculate_fletcher16(payload, sizeof(ALLb));

    uint8_t footer[2];
    footer[0] = static_cast<uint8_t>(checksum & 0xFF);
    footer[1] = static_cast<uint8_t>((checksum >> 8) & 0xFF);

    port->write(header, 4);
    port->write(payload, sizeof(ALLb));
    port->write(footer, 2);
}

uint16_t RPI::calculate_fletcher16(const uint8_t* data, size_t length) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    for (size_t i = 0; i < length; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}
