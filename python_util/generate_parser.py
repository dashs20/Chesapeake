import re
import os

def generate_layout():
    struct_defs = {}

    def parse_header(file_path):
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Strip comments
        content = re.sub(r'//.*', '', content)
        content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
        
        # Find all struct definitions
        struct_matches = re.finditer(r'struct\s+(\w+)\s*\{(.*?)\};', content, flags=re.DOTALL)
        for match in struct_matches:
            struct_name = match.group(1)
            body = match.group(2)
            
            # Parse fields
            fields = []
            field_matches = re.finditer(r'([\w\:\<\>]+)\s+(\w+)(?:\s*=\s*[^;]+)?\s*;', body)
            for f_match in field_matches:
                f_type = f_match.group(1)
                f_name = f_match.group(2)
                fields.append((f_type, f_name))
            
            struct_defs[struct_name] = fields

    # Paths to header files
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parse_header(os.path.join(project_root, "src", "HAL", "bus.hpp"))
    parse_header(os.path.join(project_root, "src", "GNC", "bus.hpp"))
    parse_header(os.path.join(project_root, "src", "CFG_APP", "bus.hpp"))

    # Primitive types dictionary (alignment, size)
    primitives = {
        'float': (4, 4),
        'bool': (1, 1),
        'uint8_t': (1, 1),
        'uint16_t': (2, 2),
        'uint32_t': (4, 4),
        'int': (4, 4),
        'FLIGHT_MODE': (4, 4),
        'Eigen::Vector3f': (4, 12),
        'Eigen::Vector2f': (4, 8),
        'Eigen::Quaternionf': (16, 16)
    }

    layout_cache = {}

    def get_layout(type_name):
        if type_name in primitives:
            return primitives[type_name] + (None,)
        
        if type_name in layout_cache:
            return layout_cache[type_name]
        
        if type_name not in struct_defs:
            # Check namespace prefix
            clean_name = type_name.split("::")[-1]
            if clean_name in struct_defs:
                type_name = clean_name
            else:
                raise ValueError(f"Unknown type: {type_name}")
                
        fields = struct_defs[type_name]
        current_offset = 0
        struct_alignment = 1
        fields_layout = []
        
        for f_type, f_name in fields:
            f_align, f_size, _ = get_layout(f_type)
            struct_alignment = max(struct_alignment, f_align)
            # Align offset
            current_offset = ((current_offset + f_align - 1) // f_align) * f_align
            fields_layout.append((f_type, f_name, current_offset))
            current_offset += f_size
            
        struct_size = ((current_offset + struct_alignment - 1) // struct_alignment) * struct_alignment
        layout_cache[type_name] = (struct_alignment, struct_size, fields_layout)
        return struct_alignment, struct_size, fields_layout

    # Flattened field offsets dictionary
    flat_offsets = {}

    def compute_flat_offsets(type_name, parent_path="", base_offset=0):
        align, size, fields = get_layout(type_name)
        if fields is None:
            # Check if this type can be broken down (e.g. Eigen vectors)
            if type_name == 'Eigen::Vector3f':
                flat_offsets[f"{parent_path}.x"] = base_offset
                flat_offsets[f"{parent_path}.y"] = base_offset + 4
                flat_offsets[f"{parent_path}.z"] = base_offset + 8
            elif type_name == 'Eigen::Vector2f':
                flat_offsets[f"{parent_path}.x"] = base_offset
                flat_offsets[f"{parent_path}.y"] = base_offset + 4
            elif type_name == 'Eigen::Quaternionf':
                flat_offsets[f"{parent_path}.x"] = base_offset
                flat_offsets[f"{parent_path}.y"] = base_offset + 4
                flat_offsets[f"{parent_path}.z"] = base_offset + 8
                flat_offsets[f"{parent_path}.w"] = base_offset + 12
            else:
                flat_offsets[parent_path] = base_offset
            return
        
        for f_type, f_name, f_offset in fields:
            path = f"{parent_path}.{f_name}" if parent_path else f_name
            compute_flat_offsets(f_type, path, base_offset + f_offset)

    compute_flat_offsets("ALLb")
    _, allb_size, _ = get_layout("ALLb")

    # Mappings from app.js key names to computed C++ path properties
    field_mapping = {
        "vbat": "halb.vbat_volts",
        "rcArm": "halb.rcrxb.arm_frac",
        "rcMod": "halb.rcrxb.mode_frac",
        "rcThr": "halb.rcrxb.thr_frac",
        "rcRol": "halb.rcrxb.roll_frac",
        "rcPit": "halb.rcrxb.pitch_frac",
        "rcYaw": "halb.rcrxb.yaw_frac",
        "armed": "gncb.vsmb.armed",
        "mode": "gncb.vsmb.mode",
        "m1": "gncb.actb.m1_frac",
        "m2": "gncb.actb.m2_frac",
        "m3": "gncb.actb.m3_frac",
        "m4": "gncb.actb.m4_frac",
        "s1": "gncb.actb.s1_deg",
        "s2": "gncb.actb.s2_deg",
        "s3": "gncb.actb.s3_deg",
        "s4": "gncb.actb.s4_deg",
        "gx": "gncb.navb.omega_body_radps.x",
        "gy": "gncb.navb.omega_body_radps.y",
        "gz": "gncb.navb.omega_body_radps.z",
        "ax": "halb.imub.accel_body_mps2.x",
        "ay": "halb.imub.accel_body_mps2.y",
        "az": "halb.imub.accel_body_mps2.z",
        "halTime": "halb.execution_time_ms",
        "timeImu": "halb.time_imu_us",
        "timeRcrx": "halb.time_rcrx_us",
        "timeMotors": "halb.time_motors_us",
        "timeServos": "halb.time_servos_us",
        "gncTime": "gncb.gnc_time_ms",
        "timeNav": "gncb.time_nav_us",
        "timeCtl": "gncb.time_ctl_us",
        "timeAlloc": "gncb.time_alloc_us",
        "cgx": "gncb.guib.omega_body_radps.x",
        "cgy": "gncb.guib.omega_body_radps.y",
        "cgz": "gncb.guib.omega_body_radps.z",
        "effRol": "gncb.ctlb.axes_effort_frac.x",
        "effPit": "gncb.ctlb.axes_effort_frac.y",
        "effYaw": "gncb.ctlb.axes_effort_frac.z",
        "isCalibrating": "cfg_appb.is_calibrating",
        "progress": "cfg_appb.calibration_progress_frac"
    }

    # Resolve JS layout object
    js_layout = {}
    for js_key, cpp_path in field_mapping.items():
        if cpp_path in flat_offsets:
            js_layout[js_key] = flat_offsets[cpp_path]
        else:
            raise KeyError(f"Mapped path '{cpp_path}' not found in flat offsets!")

    # Write telemetry_layout.js
    output_path = os.path.join(project_root, "configurator", "telemetry_layout.js")
    with open(output_path, "w") as f:
        f.write("// Automatically generated by generate_parser.py. DO NOT EDIT.\n\n")
        f.write("const ALLb_LAYOUT = {\n")
        for key, offset in sorted(js_layout.items()):
            f.write(f"    {key}: {offset},\n")
        f.write("};\n\n")
        f.write(f"const ALLb_SIZE = {allb_size};\n")

    print(f"Successfully generated telemetry_layout.js at {output_path} (ALLb struct size: {allb_size} bytes)")

if __name__ == "__main__":
    generate_layout()
