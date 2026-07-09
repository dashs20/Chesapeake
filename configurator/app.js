let port = null;
let reader = null;
let writer = null;
let isConnected = false;
let isConnecting = false;
let readLoopPromise = null;
let reloadTimeout = null;
let paramsCache = {};
let modifiedParams = {};
let serialBuffer = "";
let pendingCommandType = null; // 'dump', 'save', 'defaults', 'reboot'

// UKF Visualizer & Telemetry globals
let canvas = null;
let ctx = null;
let currentQuaternion = [1, 0, 0, 0]; // [w, x, y, z]
let isTelemetryActive = false;
let lastTelemetryTime = 0;
let animationAngle = 0;

const groupMappings = {
    "mot_": "Motors",
    "servo_": "Servos",
    "rcrx_": "RC Receiver",
    "bat_": "Battery & ADC",
    "imu_": "IMU Sensor",
    "gnc_": "Flight Loop",
    "angle_": "Attitude Loop",
    "roll_": "Roll PID Tuning",
    "pitch_": "Pitch PID Tuning",
    "yaw_": "Yaw PID Tuning",
    "blink_": "LED Indicators"
};

const dropdownOptions = {
    "imu_accel_fs": ["2G", "4G", "8G", "16G"],
    "imu_gyro_fs": ["125DPS", "250DPS", "500DPS", "1000DPS", "2000DPS", "4000DPS"],
    "imu_accel_odr": ["OFF", "1.875Hz", "7.5Hz", "15Hz", "30Hz", "60Hz", "120Hz", "240Hz", "480Hz", "960Hz", "1920Hz", "3840Hz", "7680Hz"],
    "imu_gyro_odr": ["OFF", "1.875Hz", "7.5Hz", "15Hz", "30Hz", "60Hz", "120Hz", "240Hz", "480Hz", "960Hz", "1920Hz", "3840Hz", "7680Hz"]
};

document.addEventListener("DOMContentLoaded", () => {
    const safeAddListener = (id, event, handler) => {
        const el = document.getElementById(id);
        if (el) el.addEventListener(event, handler);
    };

    safeAddListener("btn-connect", "click", toggleConnection);
    safeAddListener("btn-refresh", "click", reloadParams);
    safeAddListener("btn-save", "click", saveParamsToBoard);
    safeAddListener("btn-reboot", "click", rebootBoard);
    safeAddListener("btn-defaults", "click", resetToDefaults);
    safeAddListener("param-search", "input", filterParameters);
    safeAddListener("cli-input", "keypress", handleCliKeyPress);
    safeAddListener("btn-send-cli", "click", sendCliCommand);
    safeAddListener("btn-calibrate", "click", calibrateUKF);
    
    // Initialize UKF 3D Visualizer
    initVisualizer();
});

function switchTab(tabId) {
    document.querySelectorAll(".nav-item").forEach(item => {
        item.classList.remove("active");
    });
    document.querySelectorAll(".tab-pane").forEach(pane => {
        pane.classList.remove("active");
    });

    const activeLink = Array.from(document.querySelectorAll(".nav-item")).find(
        item => item.getAttribute("onclick").includes(tabId)
    );
    if (activeLink) activeLink.classList.add("active");

    const activePane = document.getElementById(`tab-${tabId}`);
    if (activePane) activePane.classList.add("active");
}

async function toggleConnection() {
    if (isConnecting) return;
    const btn = document.getElementById("btn-connect");
    if (btn) btn.disabled = true;
    
    isConnecting = true;
    try {
        if (isConnected) {
            await disconnect();
        } else {
            await connect();
        }
    } catch (err) {
        console.error("Toggle connection failed:", err);
    } finally {
        isConnecting = false;
        if (btn) btn.disabled = false;
    }
}

async function connect() {
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        isConnected = true;
        
        writer = port.writable.getWriter();
        reader = port.readable.getReader();

        updateConnectionUI(true);
        readLoopPromise = readLoop();

        setTimeout(async () => {
            try {
                await writeRaw("\n");
                await reloadParams();
            } catch (err) {
                console.error("Initialization failed:", err);
                await disconnect();
            }
        }, 500);

    } catch (err) {
        console.error("Connection failed:", err);
        alert("Failed to connect: " + err.message);
        updateConnectionUI(false);
        await disconnect();
    }
}

async function disconnect() {
    isConnected = false;
    
    if (reader) {
        try {
            await reader.cancel();
        } catch (e) {}
        if (readLoopPromise) {
            try {
                await readLoopPromise;
            } catch (e) {}
            readLoopPromise = null;
        }
        try {
            reader.releaseLock();
        } catch (e) {}
        reader = null;
    }
    
    if (writer) {
        try {
            writer.releaseLock();
        } catch (e) {}
        writer = null;
    }

    if (port) {
        try {
            await port.close();
        } catch (e) {}
        port = null;
    }

    updateConnectionUI(false);
    clearBoardUI();
}

function updateConnectionUI(connected) {
    const btn = document.getElementById("btn-connect");
    const status = document.getElementById("connection-status");
    const controls = ["btn-refresh", "btn-save", "btn-reboot", "btn-defaults", "cli-input", "btn-send-cli", "btn-calibrate"];

    if (btn) {
        if (connected) {
            btn.textContent = "Disconnect";
            btn.classList.add("connected");
        } else {
            btn.textContent = "Connect";
            btn.classList.remove("connected");
        }
    }
    
    if (status) {
        if (connected) {
            status.textContent = "Connected";
            status.className = "status-indicator connected";
        } else {
            status.textContent = "Disconnected";
            status.className = "status-indicator disconnected";
        }
    }

    controls.forEach(id => {
        const el = document.getElementById(id);
        if (el) {
            el.disabled = !connected;
        }
    });
}

function clearBoardUI() {
    paramsCache = {};
    modifiedParams = {};
    
    const loading = document.getElementById("params-loading");
    if (loading) loading.style.display = "block";
    
    const grid = document.getElementById("params-grid");
    if (grid) {
        grid.style.display = "none";
        grid.innerHTML = "";
    }
    
    const summaryIds = ["sum-m1", "sum-m2", "sum-m3", "sum-m4", "sum-looprate", "sum-attitude"];
    summaryIds.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.textContent = "-";
    });

    // Clear telemetry display
    updateTelemetryUI(NaN, NaN, NaN, NaN, NaN);
}

async function writeRaw(text) {
    if (!writer) return;
    const encoder = new TextEncoder();
    await writer.write(encoder.encode(text));
}

async function readLoop() {
    const decoder = new TextDecoder();
    try {
        while (isConnected && port && port.readable) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value) {
                const chunk = decoder.decode(value);
                handleIncomingChunk(chunk);
            }
        }
    } catch (err) {
        console.error("Read loop encountered error:", err);
        await disconnect();
    }
}

function handleIncomingChunk(chunk) {
    serialBuffer += chunk;
    
    let lineEndIdx;
    let cliOutputAccumulator = "";
    let telemetryUpdated = false;
    let latestTelemetry = null;

    while ((lineEndIdx = serialBuffer.indexOf("\n")) !== -1) {
        const line = serialBuffer.substring(0, lineEndIdx);
        serialBuffer = serialBuffer.substring(lineEndIdx + 1);
        
        const trimmed = line.trim();
        if (trimmed.startsWith("$TEL,")) {
            const parts = trimmed.split(",");
            if (parts.length >= 6) {
                const vbat = parseFloat(parts[1]);
                const qw = parseFloat(parts[2]);
                const qx = parseFloat(parts[3]);
                const qy = parseFloat(parts[4]);
                const qz = parseFloat(parts[5]);
                
                latestTelemetry = { vbat, qw, qx, qy, qz };
                telemetryUpdated = true;
            }
        } else {
            // Batch CLI output lines
            cliOutputAccumulator += line + "\n";
            
            if (pendingCommandType === "dump") {
                if (trimmed === "--- END OF DUMP ---") {
                    pendingCommandType = null;
                    if (reloadTimeout) {
                        clearTimeout(reloadTimeout);
                        reloadTimeout = null;
                    }
                    buildParametersUI();
                    updateSummaryUI();
                } else {
                    parseDumpLine(trimmed);
                }
            }
        }
    }

    if (telemetryUpdated && latestTelemetry) {
        updateTelemetryUI(latestTelemetry.vbat, latestTelemetry.qw, latestTelemetry.qx, latestTelemetry.qy, latestTelemetry.qz);
    }

    // Perform a single, fast DOM update for the entire chunk
    if (cliOutputAccumulator) {
        appendCliOutput(cliOutputAccumulator);
    }
}

function parseDumpLine(line) {
    const match = line.match(/^([a-zA-Z0-9_]+)\s*=\s*([^\s]+)/);
    if (match) {
        const key = match[1];
        const val = match[2];
        paramsCache[key] = val;
    }
}

async function reloadParams() {
    paramsCache = {};
    modifiedParams = {};
    pendingCommandType = "dump";
    
    const loading = document.getElementById("params-loading");
    if (loading) {
        loading.textContent = "Loading parameters from flight controller...";
        loading.style.display = "block";
    }
    const grid = document.getElementById("params-grid");
    if (grid) grid.style.display = "none";

    if (reloadTimeout) {
        clearTimeout(reloadTimeout);
    }

    await writeRaw("dump\n");

    reloadTimeout = setTimeout(() => {
        if (pendingCommandType === "dump") {
            pendingCommandType = null;
            buildParametersUI();
            updateSummaryUI();
        }
    }, 2000);
}

function buildParametersUI() {
    const grid = document.getElementById("params-grid");
    if (!grid) return;
    grid.innerHTML = "";
    
    if (Object.keys(paramsCache).length === 0) {
        const loading = document.getElementById("params-loading");
        if (loading) loading.textContent = "Failed to load parameters. Try refreshing.";
        return;
    }
    
    const loading = document.getElementById("params-loading");
    if (loading) loading.style.display = "none";
    grid.style.display = "grid";

    const groups = {};
    for (const key of Object.keys(paramsCache).sort()) {
        let groupName = "General Settings";
        for (const [prefix, name] of Object.entries(groupMappings)) {
            if (key.startsWith(prefix)) {
                groupName = name;
                break;
            }
        }
        if (!groups[groupName]) groups[groupName] = [];
        groups[groupName].push(key);
    }

    for (const [groupTitle, keys] of Object.entries(groups)) {
        const groupDiv = document.createElement("div");
        groupDiv.className = "param-group";
        
        const titleDiv = document.createElement("div");
        titleDiv.className = "param-group-title";
        titleDiv.textContent = groupTitle;
        groupDiv.appendChild(titleDiv);
        
        const rowsDiv = document.createElement("div");
        rowsDiv.className = "param-rows";
        
        keys.forEach(key => {
            const rowDiv = document.createElement("div");
            rowDiv.className = "param-row";
            
            const nameSpan = document.createElement("span");
            nameSpan.className = "param-name";
            nameSpan.textContent = key;
            rowDiv.appendChild(nameSpan);
            
            const val = paramsCache[key];
            if (dropdownOptions[key]) {
                const select = document.createElement("select");
                select.className = "param-select";
                dropdownOptions[key].forEach(opt => {
                    const option = document.createElement("option");
                    option.value = opt;
                    option.textContent = opt;
                    if (opt === val) option.selected = true;
                    select.appendChild(option);
                });
                select.addEventListener("change", (e) => {
                    handleParamChange(key, e.target.value);
                });
                rowDiv.appendChild(select);
            } else {
                const input = document.createElement("input");
                input.className = "param-input";
                input.value = val;
                input.addEventListener("input", (e) => {
                    handleParamChange(key, e.target.value);
                });
                rowDiv.appendChild(input);
            }
            rowsDiv.appendChild(rowDiv);
        });
        groupDiv.appendChild(rowsDiv);
        grid.appendChild(groupDiv);
    }
}

function handleParamChange(key, val) {
    if (paramsCache[key] === val) {
        delete modifiedParams[key];
    } else {
        modifiedParams[key] = val;
    }
}

function updateSummaryUI() {
    const safeSetText = (id, text) => {
        const el = document.getElementById(id);
        if (el) el.textContent = text;
    };
    safeSetText("sum-m1", paramsCache["mot_m1_pin"] || "-");
    safeSetText("sum-m2", paramsCache["mot_m2_pin"] || "-");
    safeSetText("sum-m3", paramsCache["mot_m3_pin"] || "-");
    safeSetText("sum-m4", paramsCache["mot_m4_pin"] || "-");
    safeSetText("sum-looprate", paramsCache["gnc_looprate_hz"] ? `${paramsCache["gnc_looprate_hz"]} Hz` : "-");
    safeSetText("sum-attitude", paramsCache["angle_loop_hz"] ? `${paramsCache["angle_loop_hz"]} Hz` : "-");
}

function filterParameters(e) {
    const query = e.target.value.toLowerCase();
    document.querySelectorAll(".param-row").forEach(row => {
        const name = row.querySelector(".param-name").textContent.toLowerCase();
        if (name.includes(query)) {
            row.style.display = "flex";
        } else {
            row.style.display = "none";
        }
    });

    document.querySelectorAll(".param-group").forEach(group => {
        const visibleRows = Array.from(group.querySelectorAll(".param-row")).filter(r => r.style.display !== "none");
        group.style.display = visibleRows.length > 0 ? "block" : "none";
    });
}

async function saveParamsToBoard() {
    const keys = Object.keys(modifiedParams);
    if (keys.length === 0) {
        alert("No parameters have been modified.");
        return;
    }

    for (const key of keys) {
        const val = modifiedParams[key];
        await writeRaw(`set ${key} = ${val}\n`);
        await new Promise(r => setTimeout(r, 100));
    }

    await writeRaw("save\n");
    alert("Configurations written successfully. The flight controller will now reboot.");
    await disconnect();
}

async function rebootBoard() {
    if (!confirm("Are you sure you want to reboot the flight controller?")) return;
    await writeRaw("reboot\n");
    await disconnect();
}

async function resetToDefaults() {
    if (!confirm("Reset all settings to default values? This will erase RAM settings and reboot the board.")) return;
    await writeRaw("defaults\n");
    await new Promise(r => setTimeout(r, 200));
    await writeRaw("save\n");
    await disconnect();
}

function handleCliKeyPress(e) {
    if (e.key === "Enter") {
        sendCliCommand();
    }
}

async function sendCliCommand() {
    const input = document.getElementById("cli-input");
    const cmd = input.value.trim();
    if (!cmd) return;
    
    appendCliOutput(`\n> ${cmd}\n`);
    await writeRaw(cmd + "\n");
    input.value = "";
}

function appendCliOutput(text) {
    const output = document.getElementById("cli-output");
    output.textContent += text;
    output.scrollTop = output.scrollHeight;
}

// Telemetry & UKF 3D Visualizer functions
function updateTelemetryUI(vbat, qw, qx, qy, qz) {
    const vbatStr = isNaN(vbat) ? "- V" : `${vbat.toFixed(2)} V`;
    const infoBat = document.getElementById("info-battery");
    const telBat = document.getElementById("tel-vbat");
    if (infoBat) infoBat.textContent = vbatStr;
    if (telBat) telBat.textContent = vbatStr;
    
    const telQw = document.getElementById("tel-qw");
    const telQx = document.getElementById("tel-qx");
    const telQy = document.getElementById("tel-qy");
    const telQz = document.getElementById("tel-qz");
    
    if (telQw) telQw.textContent = isNaN(qw) ? "-" : qw.toFixed(4);
    if (telQx) telQx.textContent = isNaN(qx) ? "-" : qx.toFixed(4);
    if (telQy) telQy.textContent = isNaN(qy) ? "-" : qy.toFixed(4);
    if (telQz) telQz.textContent = isNaN(qz) ? "-" : qz.toFixed(4);
    
    if (!isNaN(qw) && !isNaN(qx) && !isNaN(qy) && !isNaN(qz)) {
        currentQuaternion = [qw, qx, qy, qz];
        isTelemetryActive = true;
        lastTelemetryTime = Date.now();
    } else {
        isTelemetryActive = false;
    }
}

function initVisualizer() {
    canvas = document.getElementById("ukf-canvas");
    if (canvas) {
        ctx = canvas.getContext("2d");
        requestAnimationFrame(drawScene);
    }
}

function rotateVectorByQuaternion(v, q) {
    const w = q[0], qx = q[1], qy = q[2], qz = q[3];
    const vx = v[0], vy = v[1], vz = v[2];

    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);

    const vpx = vx + w * tx + (qy * tz - qz * ty);
    const vpy = vy + w * ty + (qz * tx - qx * tz);
    const vpz = vz + w * tz + (qx * ty - qy * tx);

    return [vpx, vpy, vpz];
}

function eulerToQuaternion(roll, pitch, yaw) {
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    const cy = Math.cos(yaw * 0.5);
    const sy = Math.sin(yaw * 0.5);

    const w = cr * cp * cy + sr * sp * sy;
    const x = sr * cp * cy - cr * sp * sy;
    const y = cr * sp * cy + sr * cp * sy;
    const z = cr * cp * sy - sr * sp * cy;

    return [w, x, y, z];
}

function rotateCamera(v) {
    // Elevate and turn slightly for nice 3D perspective
    const yaw = -35 * Math.PI / 180;
    const pitch = -20 * Math.PI / 180;
    
    // Rotate around Z (Yaw)
    const x1 = v[0] * Math.cos(yaw) - v[1] * Math.sin(yaw);
    const y1 = v[0] * Math.sin(yaw) + v[1] * Math.cos(yaw);
    const z1 = v[2];
    
    // Rotate around X (Pitch)
    const x2 = x1;
    const y2 = y1 * Math.cos(pitch) - z1 * Math.sin(pitch);
    const z2 = y1 * Math.sin(pitch) + z1 * Math.cos(pitch);
    
    return [x2, y2, z2];
}

function drawScene() {
    if (!ctx) return;
    
    const setupTab = document.getElementById("tab-setup");
    if (setupTab && !setupTab.classList.contains("active")) {
        setTimeout(() => {
            requestAnimationFrame(drawScene);
        }, 200);
        return;
    }
    
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    let q = [1, 0, 0, 0];
    if (isConnected && isTelemetryActive && (Date.now() - lastTelemetryTime < 2000)) {
        // Use active telemetry quaternion. We take the conjugate (w, -x, -y, -z)
        // because the estimated quaternion q_earth2body rotates earth coordinates to body,
        // while we want to orient the body relative to the earth/screen.
        q = [currentQuaternion[0], -currentQuaternion[1], -currentQuaternion[2], -currentQuaternion[3]];
    } else {
        // Idle animation
        animationAngle += 0.008;
        q = eulerToQuaternion(animationAngle * 0.4, animationAngle, animationAngle * 0.6);
    }
    
    const width = canvas.width;
    const height = canvas.height;
    const scale = 55;
    
    // 3D block representing the drone body
    const baseVertices = [
        [-1.0, -0.6, -0.2], // 0: back-left-top
        [ 1.0, -0.6, -0.2], // 1: front-left-top
        [ 1.0,  0.6, -0.2], // 2: front-right-top
        [-1.0,  0.6, -0.2], // 3: back-right-top
        [-1.0, -0.6,  0.2], // 4: back-left-bottom
        [ 1.0, -0.6,  0.2], // 5: front-left-bottom
        [ 1.0,  0.6,  0.2], // 6: front-right-bottom
        [-1.0,  0.6,  0.2]  // 7: back-right-bottom
    ];
    
    const projected = [];
    for (let i = 0; i < baseVertices.length; i++) {
        let v = [baseVertices[i][0] * scale, baseVertices[i][1] * scale, baseVertices[i][2] * scale];
        let r = rotateVectorByQuaternion(v, q);
        let c = rotateCamera(r);
        
        // Project to 2D
        const x2d = c[0] + width / 2;
        const y2d = -c[1] + height / 2; // Negate Y so positive is up in 3D space
        const depth = c[2];
        projected.push({ x: x2d, y: y2d, z: depth });
    }
    
    const faces = [
        { indices: [1, 2, 6, 5], color: "rgba(234, 179, 8, 0.35)", borderColor: "#eab308", label: "FRONT" },
        { indices: [0, 3, 7, 4], color: "rgba(71, 85, 105, 0.3)", borderColor: "#475569" },
        { indices: [0, 1, 5, 4], color: "rgba(71, 85, 105, 0.3)", borderColor: "#475569" },
        { indices: [2, 3, 7, 6], color: "rgba(71, 85, 105, 0.3)", borderColor: "#475569" },
        { indices: [0, 1, 2, 3], color: "rgba(30, 41, 59, 0.4)", borderColor: "#334155" },
        { indices: [4, 5, 6, 7], color: "rgba(30, 41, 59, 0.4)", borderColor: "#334155" }
    ];
    
    faces.forEach(face => {
        face.avgDepth = face.indices.reduce((sum, idx) => sum + projected[idx].z, 0) / 4;
    });
    faces.sort((a, b) => b.avgDepth - a.avgDepth);
    
    // Draw faces using painter's algorithm (furthest first)
    faces.forEach(face => {
        ctx.beginPath();
        ctx.moveTo(projected[face.indices[0]].x, projected[face.indices[0]].y);
        for (let i = 1; i < face.indices.length; i++) {
            ctx.lineTo(projected[face.indices[i]].x, projected[face.indices[i]].y);
        }
        ctx.closePath();
        
        ctx.fillStyle = face.color;
        ctx.fill();
        ctx.strokeStyle = face.borderColor;
        ctx.lineWidth = 1.5;
        ctx.stroke();
        
        if (face.label) {
            const cx = face.indices.reduce((sum, idx) => sum + projected[idx].x, 0) / 4;
            const cy = face.indices.reduce((sum, idx) => sum + projected[idx].y, 0) / 4;
            ctx.fillStyle = "#ffffff";
            ctx.font = "bold 9px 'Outfit', sans-serif";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText(face.label, cx, cy);
        }
    });
    
    // Draw coordinate axes
    const axes = [
        { dir: [1.8, 0, 0], color: "#ef4444", label: "X" }, // Forward
        { dir: [0, 1.8, 0], color: "#10b981", label: "Y" }, // Right
        { dir: [0, 0, -1.8], color: "#3b82f6", label: "Z" } // Up (negative Z in NED)
    ];
    
    axes.forEach(axis => {
        const originBody = [0, 0, 0];
        const tipBody = [axis.dir[0] * scale, axis.dir[1] * scale, axis.dir[2] * scale];
        
        const originRot = rotateVectorByQuaternion(originBody, q);
        const tipRot = rotateVectorByQuaternion(tipBody, q);
        
        const originCam = rotateCamera(originRot);
        const tipCam = rotateCamera(tipRot);
        
        const x1 = originCam[0] + width / 2;
        const y1 = -originCam[1] + height / 2;
        const x2 = tipCam[0] + width / 2;
        const y2 = -tipCam[1] + height / 2;
        
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.strokeStyle = axis.color;
        ctx.lineWidth = 2;
        ctx.stroke();
        
        ctx.fillStyle = axis.color;
        ctx.font = "bold 10px 'Outfit', sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(axis.label, x2 + (x2 - x1) * 0.15, y2 + (y2 - y1) * 0.15);
    });
    
    requestAnimationFrame(drawScene);
}

async function calibrateUKF() {
    if (!isConnected) return;
    const btn = document.getElementById("btn-calibrate");
    btn.disabled = true;
    btn.textContent = "Calibrating...";
    
    appendCliOutput("\n> calibrate\n");
    await writeRaw("calibrate\n");
    
    // The board will output instructions, delay, and reboot.
    // We will automatically restore the button text after a delay,
    // although the board will disconnect shortly after anyway.
    setTimeout(() => {
        btn.textContent = "Calibrate UKF (Flat)";
        if (isConnected) {
            btn.disabled = false;
        }
    }, 4000);
}

