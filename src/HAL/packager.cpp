#include "packager.hpp"
#include <cstring>

uint16_t Packager::calculate_fletcher16(const uint8_t* data, size_t length) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    for (size_t i = 0; i < length; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

size_t Packager::package_allb(const ALLb& allb, uint8_t* buffer, size_t max_len) {
    size_t payload_len = sizeof(ALLb);
    size_t total_len = 5 + payload_len + 2;
    if (max_len < total_len) {
        return 0;
    }

    buffer[0] = 0xAA;
    buffer[1] = 0xBB;
    buffer[2] = 0x01;
    buffer[3] = payload_len & 0xFF;
    buffer[4] = (payload_len >> 8) & 0xFF;

    std::memcpy(buffer + 5, &allb, payload_len);

    uint16_t checksum = calculate_fletcher16(reinterpret_cast<const uint8_t*>(&allb), payload_len);
    buffer[5 + payload_len] = checksum & 0xFF;
    buffer[5 + payload_len + 1] = (checksum >> 8) & 0xFF;

    return total_len;
}
