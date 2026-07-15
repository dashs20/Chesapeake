#pragma once

#include <Arduino.h>
#include <flatbuffers/flatbuffers.h>

namespace PKG {

template <typename T>
void send_packet(const T& data, Stream* port) {
    if (port == nullptr) {
        return;
    }

    flatbuffers::FlatBufferBuilder builder(sizeof(T) + 64);
    auto vec_offset = builder.CreateVector(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
    
    flatbuffers::uoffset_t start = builder.StartTable();
    builder.AddOffset(4, vec_offset);
    flatbuffers::uoffset_t end = builder.EndTable(start);
    builder.Finish(flatbuffers::Offset<void>(end));

    uint16_t fb_len = builder.GetSize();

    if (port == &Serial) {
        if (!Serial) {
            return;
        }
    }

    uint8_t header[5];
    header[0] = 0xAA;
    header[1] = 0xBB;
    header[2] = 0x01;
    header[3] = fb_len & 0xFF;
    header[4] = (fb_len >> 8) & 0xFF;

    port->write(header, 5);
    port->write(builder.GetBufferPointer(), fb_len);
}

} // namespace PKG
