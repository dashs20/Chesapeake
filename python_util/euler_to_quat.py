import math

def main():
    # Input Euler angles in degrees (X, Y, Z sequential)
    # Edit these values and run the script
    x = 180.0
    y = 0.0
    z = 90.0

    roll = math.radians(x)
    pitch = math.radians(y)
    yaw = math.radians(z)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x_val = sr * cp * cy - cr * sp * sy
    y_val = cr * sp * cy + sr * cp * sy
    z_val = cr * cp * sy - sr * sp * cy

    print(f"Input Euler Angles (X, Y, Z): {x}, {y}, {z} degrees")
    print(f"Format (w, x, y, z): ({w:.6f}, {x_val:.6f}, {y_val:.6f}, {z_val:.6f})")
    print(f"Format (x, y, z, w): ({x_val:.6f}, {y_val:.6f}, {z_val:.6f}, {w:.6f})")

if __name__ == "__main__":
    main()
