import argparse
import math
import serial
import struct
import time

def parse_flatbuffer(payload):
    # Root offset
    root_offset = struct.unpack('<I', payload[0:4])[0]
    # Vtable offset
    vtable_offset = struct.unpack('<i', payload[root_offset:root_offset+4])[0]
    vtable_start = root_offset - vtable_offset
    # Field index 0 maps to offset 4 in vtable
    field_offset = struct.unpack('<H', payload[vtable_start+4:vtable_start+6])[0]
    if field_offset == 0:
        return None
    # Vector offset
    vector_start = root_offset + field_offset + struct.unpack('<i', payload[root_offset+field_offset:root_offset+field_offset+4])[0]
    vector_len = struct.unpack('<I', payload[vector_start:vector_start+4])[0]
    return payload[vector_start+4 : vector_start+4+vector_len]

def collect_samples(ser, n_samples):
    samples = []
    buffer = bytearray()
    print(f"Collecting {n_samples} raw IMU samples. Keep the board flat and still...")
    
    while len(samples) < n_samples:
        data = ser.read(max(1, ser.in_waiting))
        if not data:
            continue
        buffer.extend(data)
        
        while True:
            idx = buffer.find(b'\xAA\xBB\x01')
            if idx == -1:
                break
            if len(buffer) < idx + 5:
                break
            
            fb_len = struct.unpack('<H', buffer[idx+3:idx+5])[0]
            total_len = 5 + fb_len
            
            if len(buffer) < idx + total_len:
                break
            
            payload = buffer[idx+5 : idx+total_len]
            struct_bytes = parse_flatbuffer(payload)
            if struct_bytes is not None and len(struct_bytes) >= 24:
                gx, gy, gz = struct.unpack('<fff', struct_bytes[0:12])
                ax, ay, az = struct.unpack('<fff', struct_bytes[12:24])
                samples.append((gx, gy, gz, ax, ay, az))
                
                if len(samples) % (n_samples // 10 or 1) == 0:
                    print(f"Progress: {len(samples)}/{n_samples} samples collected...")
            
            buffer = buffer[idx+total_len:]
            
    return samples

def main():
    parser = argparse.ArgumentParser(description="Chesapeake flight controller IMU calibration helper")
    parser.add_argument("--port", required=True, help="Serial port of the flight controller (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="CLI communication baud rate")
    parser.add_argument("--samples", type=int, default=200, help="Number of samples to collect")
    parser.add_argument("--force-zero", action="store_true", help="Unconditionally zero biases before calibration")
    args = parser.parse_args()

    print(f"Connecting to flight controller on {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    
    if args.force_zero:
        print("Zeroing biases on the board pre-calibration...")
        ser.write(b"set gnc_nav_imu_calc_accel_bias_x = 0\n")
        time.sleep(0.1)
        ser.write(b"set gnc_nav_imu_calc_accel_bias_y = 0\n")
        time.sleep(0.1)
        ser.write(b"set gnc_nav_imu_calc_accel_bias_z = 0\n")
        time.sleep(0.1)
        ser.write(b"set gnc_nav_imu_calc_gyro_bias_x = 0\n")
        time.sleep(0.1)
        ser.write(b"set gnc_nav_imu_calc_gyro_bias_y = 0\n")
        time.sleep(0.1)
        ser.write(b"set gnc_nav_imu_calc_gyro_bias_z = 0\n")
        time.sleep(0.1)
        ser.write(b"save\n")
        print("Rebooting board. Waiting 5 seconds to reconnect...")
        ser.close()
        time.sleep(5.0)
        ser = serial.Serial(args.port, args.baud, timeout=1)

    ser.reset_input_buffer()
    
    samples = collect_samples(ser, args.samples)
    
    # Process gyro data (average each axis)
    sum_gx = sum_gy = sum_gz = 0.0
    sum_ax = sum_ay = sum_az = 0.0
    for s in samples:
        sum_gx += s[0]
        sum_gy += s[1]
        sum_gz += s[2]
        sum_ax += s[3]
        sum_ay += s[4]
        sum_az += s[5]
        
    avg_gx = sum_gx / len(samples)
    avg_gy = sum_gy / len(samples)
    avg_gz = sum_gz / len(samples)
    
    avg_ax = sum_ax / len(samples)
    avg_ay = sum_ay / len(samples)
    avg_az = sum_az / len(samples)

    # Process accel data:
    # 1) Derive unit vector
    norm = math.sqrt(avg_ax**2 + avg_ay**2 + avg_az**2)
    ux = avg_ax / norm
    uy = avg_ay / norm
    uz = avg_az / norm
    
    # 2) Multiply unit vector by g (9.80665)
    g = 9.80665
    expected_ax = ux * g
    expected_ay = uy * g
    expected_az = uz * g
    
    # 3) Subtract from averages to get bias
    bias_ax = avg_ax - expected_ax
    bias_ay = avg_ay - expected_ay
    bias_az = avg_az - expected_az

    print("\nCalibration Results:")
    print(f"Gyro Biases (rad/s): X: {avg_gx:.6f}, Y: {avg_gy:.6f}, Z: {avg_gz:.6f}")
    print(f"Accel Biases (m/s^2): X: {bias_ax:.6f}, Y: {bias_ay:.6f}, Z: {bias_az:.6f}")

    # Write parameters back to flight controller
    print("\nWriting new calibration biases to flight controller RAM...")
    ser.write(f"set gnc_nav_imu_calc_accel_bias_x = {bias_ax:.6f}\n".encode())
    time.sleep(0.1)
    ser.write(f"set gnc_nav_imu_calc_accel_bias_y = {bias_ay:.6f}\n".encode())
    time.sleep(0.1)
    ser.write(f"set gnc_nav_imu_calc_accel_bias_z = {bias_az:.6f}\n".encode())
    time.sleep(0.1)
    ser.write(f"set gnc_nav_imu_calc_gyro_bias_x = {avg_gx:.6f}\n".encode())
    time.sleep(0.1)
    ser.write(f"set gnc_nav_imu_calc_gyro_bias_y = {avg_gy:.6f}\n".encode())
    time.sleep(0.1)
    ser.write(f"set gnc_nav_imu_calc_gyro_bias_z = {avg_gz:.6f}\n".encode())
    time.sleep(0.1)
    
    print("Saving parameters to LittleFS and rebooting board...")
    ser.write(b"save\n")
    time.sleep(0.5)
    ser.close()
    print("Calibration successful and complete.")

if __name__ == "__main__":
    main()
