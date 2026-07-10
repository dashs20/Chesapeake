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
let pendingCommandType = null;
let isCollectingCalibration = false;
let collectedSamples = [];

const dropdownOptions = {
    "imu_accel_fs": ["2G", "4G", "8G", "16G"],
    "imu_gyro_fs": ["125DPS", "250DPS", "500DPS", "1000DPS", "2000DPS", "4000DPS"],
    "imu_accel_odr": ["OFF", "1.875Hz", "7.5Hz", "15Hz", "30Hz", "60Hz", "120Hz", "240Hz", "480Hz", "960Hz", "1920Hz", "3840Hz", "7680Hz"],
    "imu_gyro_odr": ["OFF", "1.875Hz", "7.5Hz", "15Hz", "30Hz", "60Hz", "120Hz", "240Hz", "480Hz", "960Hz", "1920Hz", "3840Hz", "7680Hz"]
};

document.addEventListener("DOMContentLoaded", () => {
    document.getElementById("btn-connect").addEventListener("click", toggleConnection);
    document.getElementById("btn-refresh").addEventListener("click", reloadParams);
    document.getElementById("btn-save").addEventListener("click", saveParamsToBoard);
    document.getElementById("btn-reboot").addEventListener("click", rebootBoard);
    document.getElementById("btn-defaults").addEventListener("click", resetToDefaults);
    document.getElementById("btn-calibrate").addEventListener("click", startCalibration);
    document.getElementById("param-search").addEventListener("input", filterParameters);
    document.getElementById("cli-input").addEventListener("keypress", (e) => {
        if (e.key === "Enter") sendCliCommand();
    });
    document.getElementById("btn-send-cli").addEventListener("click", sendCliCommand);
});

async function toggleConnection() {
    if (isConnecting) return;
    const btn = document.getElementById("btn-connect");
    btn.disabled = true;
    isConnecting = true;
    try {
        if (isConnected) {
            await disconnect();
        } else {
            await connect();
        }
    } catch (err) {
        console.error(err);
    } finally {
        isConnecting = false;
        btn.disabled = false;
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
            await writeRaw("\n");
            await reloadParams();
        }, 500);
    } catch (err) {
        alert("Failed to connect: " + err.message);
        updateConnectionUI(false);
        await disconnect();
    }
}

async function disconnect() {
    isConnected = false;
    updateConnectionUI(false);
    clearBoardUI();
    cleanupSerialPort();
}

async function cleanupSerialPort() {
    const localReader = reader;
    const localWriter = writer;
    const localPort = port;
    const localPromise = readLoopPromise;
    
    reader = null;
    writer = null;
    port = null;
    readLoopPromise = null;
    
    if (localReader) {
        try { await localReader.cancel(); } catch (e) {}
        if (localPromise) {
            try { await localPromise; } catch (e) {}
        }
        try { localReader.releaseLock(); } catch (e) {}
    }
    if (localWriter) {
        try { localWriter.releaseLock(); } catch (e) {}
    }
    if (localPort) {
        try { await localPort.close(); } catch (e) {}
    }
}

function updateConnectionUI(connected) {
    const btn = document.getElementById("btn-connect");
    const status = document.getElementById("connection-status");
    const controls = ["btn-refresh", "btn-save", "btn-reboot", "btn-defaults", "btn-calibrate", "cli-input", "btn-send-cli", "param-search"];
    btn.textContent = connected ? "Disconnect" : "Connect";
    btn.className = connected ? "btn btn-connected" : "btn";
    status.textContent = connected ? "Connected" : "Disconnected";
    status.className = connected ? "status-indicator connected" : "status-indicator disconnected";
    controls.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = !connected;
    });
}

function clearBoardUI() {
    paramsCache = {};
    modifiedParams = {};
    const placeholder = document.getElementById("params-loading");
    placeholder.textContent = "Connect to view and edit flight parameters.";
    placeholder.style.display = "block";
    const grid = document.getElementById("params-grid");
    grid.style.display = "none";
    grid.innerHTML = "";
    const monitor = document.getElementById("serial-monitor");
    if (monitor) monitor.textContent = "";
    const output = document.getElementById("cli-output");
    if (output) output.textContent = "";
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
                handleIncomingChunk(decoder.decode(value));
            }
        }
    } catch (err) {
        console.error(err);
        await disconnect();
    }
}

function handleIncomingChunk(chunk) {
    serialBuffer += chunk;
    let lineEndIdx;
    let cliOutput = "";
    let rawOutput = "";
    while ((lineEndIdx = serialBuffer.indexOf("\n")) !== -1) {
        const line = serialBuffer.substring(0, lineEndIdx);
        serialBuffer = serialBuffer.substring(lineEndIdx + 1);
        const trimmed = line.trim();
        if (trimmed.startsWith("$DBG,")) {
            rawOutput += line + "\n";
            if (isCollectingCalibration) {
                const parts = trimmed.split(",");
                if (parts.length >= 9) {
                    collectedSamples.push({
                        ax: parseFloat(parts[1]),
                        ay: parseFloat(parts[2]),
                        az: parseFloat(parts[3]),
                        gx: parseFloat(parts[4]),
                        gy: parseFloat(parts[5]),
                        gz: parseFloat(parts[6])
                    });
                    const progress = Math.round((collectedSamples.length / 20) * 100);
                    document.getElementById("btn-calibrate").textContent = `Calibrating (${progress}%)...`;
                    if (collectedSamples.length >= 20) {
                        processCalibration();
                    }
                }
            }
            continue;
        }
        if (trimmed.startsWith("$")) {
            continue;
        }
        cliOutput += line + "\n";
        if (pendingCommandType === "dump") {
            if (trimmed === "--- END OF DUMP ---") {
                pendingCommandType = null;
                if (reloadTimeout) {
                    clearTimeout(reloadTimeout);
                    reloadTimeout = null;
                }
                buildParametersUI();
            } else {
                const match = trimmed.match(/^([a-zA-Z0-9_]+)\s*=\s*([^\s]+)/);
                if (match) {
                    paramsCache[match[1]] = match[2];
                }
            }
        }
    }
    if (rawOutput) {
        appendSerialMonitor(rawOutput);
    }
    if (cliOutput) {
        const output = document.getElementById("cli-output");
        output.textContent += cliOutput;
        output.scrollTop = output.scrollHeight;
    }
}

function appendSerialMonitor(text) {
    const monitor = document.getElementById("serial-monitor");
    if (!monitor) return;
    monitor.textContent += text;
    const lines = monitor.textContent.split("\n");
    if (lines.length > 150) {
        monitor.textContent = lines.slice(lines.length - 150).join("\n");
    }
    monitor.scrollTop = monitor.scrollHeight;
}

async function reloadParams() {
    paramsCache = {};
    modifiedParams = {};
    pendingCommandType = "dump";
    const placeholder = document.getElementById("params-loading");
    placeholder.textContent = "Loading parameters from flight controller...";
    placeholder.style.display = "block";
    document.getElementById("params-grid").style.display = "none";
    if (reloadTimeout) clearTimeout(reloadTimeout);
    await writeRaw("dump\n");
    reloadTimeout = setTimeout(() => {
        if (pendingCommandType === "dump") {
            pendingCommandType = null;
            buildParametersUI();
        }
    }, 2000);
}

function buildParametersUI() {
    const grid = document.getElementById("params-grid");
    const placeholder = document.getElementById("params-loading");
    grid.innerHTML = "";
    if (Object.keys(paramsCache).length === 0) {
        placeholder.textContent = "Failed to load parameters. Try reloading.";
        return;
    }
    placeholder.style.display = "none";
    grid.style.display = "flex";
    for (const key of Object.keys(paramsCache).sort()) {
        const row = document.createElement("div");
        row.className = "param-row";
        const label = document.createElement("span");
        label.className = "param-name";
        label.textContent = key;
        row.appendChild(label);
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
            row.appendChild(select);
        } else {
            const input = document.createElement("input");
            input.className = "param-input";
            input.value = val;
            input.addEventListener("input", (e) => {
                handleParamChange(key, e.target.value);
            });
            row.appendChild(input);
        }
        grid.appendChild(row);
    }
}

function handleParamChange(key, val) {
    if (paramsCache[key] === val) {
        delete modifiedParams[key];
    } else {
        modifiedParams[key] = val;
    }
}

function filterParameters(e) {
    const query = e.target.value.toLowerCase();
    document.querySelectorAll(".param-row").forEach(row => {
        const name = row.querySelector(".param-name").textContent.toLowerCase();
        row.style.display = name.includes(query) ? "flex" : "none";
    });
}

async function saveParamsToBoard() {
    const keys = Object.keys(modifiedParams);
    if (keys.length === 0) {
        alert("No parameters have been modified.");
        return;
    }
    for (const key of keys) {
        await writeRaw(`set ${key} = ${modifiedParams[key]}\n`);
        await new Promise(r => setTimeout(r, 100));
    }
    await writeRaw("save\n");
    alert("Parameters saved. Reconnecting...");
    await disconnect();
}

async function rebootBoard() {
    if (!confirm("Reboot flight controller?")) return;
    await writeRaw("reboot\n");
    await disconnect();
}

async function resetToDefaults() {
    if (!confirm("Reset to defaults?")) return;
    await writeRaw("defaults\n");
    await new Promise(r => setTimeout(r, 200));
    await writeRaw("save\n");
    await disconnect();
}

async function sendCliCommand() {
    const input = document.getElementById("cli-input");
    const cmd = input.value.trim();
    if (!cmd) return;
    const output = document.getElementById("cli-output");
    output.textContent += `\n> ${cmd}\n`;
    output.scrollTop = output.scrollHeight;
    await writeRaw(cmd + "\n");
    input.value = "";
}

function startCalibration() {
    if (!isConnected) return;
    document.getElementById("btn-calibrate").disabled = true;
    document.getElementById("btn-calibrate").textContent = "Calibrating (0%)...";
    collectedSamples = [];
    isCollectingCalibration = true;
}

async function processCalibration() {
    isCollectingCalibration = false;
    document.getElementById("btn-calibrate").textContent = "Processing...";
    
    let sum_ax = 0, sum_ay = 0, sum_az = 0;
    let sum_gx = 0, sum_gy = 0, sum_gz = 0;
    
    collectedSamples.forEach(s => {
        sum_ax += s.ax;
        sum_ay += s.ay;
        sum_az += s.az;
        sum_gx += s.gx;
        sum_gy += s.gy;
        sum_gz += s.gz;
    });
    
    const count = collectedSamples.length;
    const avg_ax = sum_ax / count;
    const avg_ay = sum_ay / count;
    const avg_az = sum_az / count;
    const avg_gx = sum_gx / count;
    const avg_gy = sum_gy / count;
    const avg_gz = sum_gz / count;
    
    const bias_ax_curr = parseFloat(paramsCache["imu_accel_bias_x_mps2"]) || 0;
    const bias_ay_curr = parseFloat(paramsCache["imu_accel_bias_y_mps2"]) || 0;
    const bias_az_curr = parseFloat(paramsCache["imu_accel_bias_z_mps2"]) || 0;
    const bias_gx_curr = parseFloat(paramsCache["imu_gyro_bias_x_radps"]) || 0;
    const bias_gy_curr = parseFloat(paramsCache["imu_gyro_bias_y_radps"]) || 0;
    const bias_gz_curr = parseFloat(paramsCache["imu_gyro_bias_z_radps"]) || 0;
    
    const raw_ax = avg_ax + bias_ax_curr;
    const raw_ay = avg_ay + bias_ay_curr;
    const raw_az = avg_az + bias_az_curr;
    const raw_gx = avg_gx + bias_gx_curr;
    const raw_gy = avg_gy + bias_gy_curr;
    const raw_gz = avg_gz + bias_gz_curr;
    
    const g = 9.80665;
    const diff_x = Math.abs(Math.abs(raw_ax) - g);
    const diff_y = Math.abs(Math.abs(raw_ay) - g);
    const diff_z = Math.abs(Math.abs(raw_az) - g);
    
    let bias_ax = raw_ax;
    let bias_ay = raw_ay;
    let bias_az = raw_az;
    
    const min_diff = Math.min(diff_x, diff_y, diff_z);
    if (min_diff === diff_x) {
        bias_ax = raw_ax > 0 ? (raw_ax - g) : (raw_ax + g);
    } else if (min_diff === diff_y) {
        bias_ay = raw_ay > 0 ? (raw_ay - g) : (raw_ay + g);
    } else {
        bias_az = raw_az > 0 ? (raw_az - g) : (raw_az + g);
    }
    
    const bias_gx = raw_gx;
    const bias_gy = raw_gy;
    const bias_gz = raw_gz;
    
    try {
        await writeRaw(`set imu_accel_bias_x_mps2 = ${bias_ax.toFixed(6)}\n`);
        await new Promise(r => setTimeout(r, 100));
        await writeRaw(`set imu_accel_bias_y_mps2 = ${bias_ay.toFixed(6)}\n`);
        await new Promise(r => setTimeout(r, 100));
        await writeRaw(`set imu_accel_bias_z_mps2 = ${bias_az.toFixed(6)}\n`);
        await new Promise(r => setTimeout(r, 100));
        await writeRaw(`set imu_gyro_bias_x_radps = ${bias_gx.toFixed(6)}\n`);
        await new Promise(r => setTimeout(r, 100));
        await writeRaw(`set imu_gyro_bias_y_radps = ${bias_gy.toFixed(6)}\n`);
        await new Promise(r => setTimeout(r, 100));
        await writeRaw(`set imu_gyro_bias_z_radps = ${bias_gz.toFixed(6)}\n`);
        await new Promise(r => setTimeout(r, 100));
        await writeRaw("save\n");
        
        alert("Calibration successful! Saved config to flash and rebooting board.");
    } catch (err) {
        alert("Calibration failed: " + err.message);
    } finally {
        document.getElementById("btn-calibrate").textContent = "Calibrate";
        document.getElementById("btn-calibrate").disabled = false;
        await disconnect();
    }
}
