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
let calibrationState = "idle";
let smoothedTotalTime = 0;
let smoothedHalTime = 0;
let smoothedGncTime = 0;
let lastLoopTimeUpdate = 0;
let smoothedTimeImu = 0;
let smoothedTimeRcrx = 0;
let smoothedTimeMotors = 0;
let smoothedTimeServos = 0;
let smoothedTimeNav = 0;
let smoothedTimeCtl = 0;
let smoothedTimeAlloc = 0;
let smoothedTimeIdle = 0;
let chartInstance = null;

let incomingBytesBuffer = new Uint8Array(0);
let textDecoder = new TextDecoder();
let textAccumulator = "";

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
    document.getElementById("btn-zero-biases").addEventListener("click", zeroBiases);
    document.getElementById("btn-toggle-test").addEventListener("click", toggleActuatorTest);
    document.getElementById("select-allocator").addEventListener("change", (e) => {
        handleParamChange("gnc_allocator", e.target.value);
    });
    document.getElementById("cli-input").addEventListener("keypress", (e) => {
        if (e.key === "Enter") sendCliCommand();
    });
    document.getElementById("btn-send-cli").addEventListener("click", sendCliCommand);
    initTabs();
    
    const canvas = document.getElementById("control-rate-chart");
    if (canvas) {
        chartInstance = new TelemetryChart("control-rate-chart");
    }
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
    if (isTestModeEnabled) {
        toggleActuatorTest();
    }
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
    const controls = ["btn-refresh", "btn-save", "btn-reboot", "btn-defaults", "btn-zero-biases", "btn-calibrate", "cli-input", "btn-send-cli", "btn-toggle-test", "select-allocator"];
    btn.textContent = connected ? "Disconnect" : "Connect";
    btn.className = connected ? "btn btn-connected" : "btn";
    status.textContent = connected ? "Connected" : "Disconnected";
    status.className = connected ? "status-indicator connected" : "status-indicator disconnected";
    controls.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = !connected;
    });
    
    const testPlaceholder = document.getElementById("test-placeholder");
    if (testPlaceholder) {
        testPlaceholder.textContent = connected ? "Enable test mode to control actuators." : "Connect and enable test mode to control actuators.";
    }
}

function clearBoardUI() {
    paramsCache = {};
    modifiedParams = {};
    const placeholder = document.getElementById("params-loading");
    if (placeholder) {
        placeholder.textContent = "Connect to view and edit flight parameters.";
        placeholder.style.display = "block";
    }

    const selAlloc = document.getElementById("select-allocator");
    if (selAlloc) {
        selAlloc.value = "QUAD";
        selAlloc.disabled = true;
    }
    ["m1", "m2", "m3", "m4"].forEach(id => {
        const elText = document.getElementById(`val-alloc-${id}`);
        const elCircle = document.getElementById(`circle-alloc-${id}`);
        if (elText) elText.textContent = "0.000000";
        if (elCircle) elCircle.style.fillOpacity = 0.1;
    });
 
    ["roll", "pitch", "yaw"].forEach(axis => {
        const elText = document.getElementById(`txt-effort-${axis}`);
        const elBar = document.getElementById(`bar-effort-${axis}`);
        if (elText) elText.textContent = "0.0000";
        if (elBar) {
            elBar.style.width = "0%";
            elBar.style.left = "50%";
        }
    });

    const pidKeys = [
        "roll_rate_kp", "roll_rate_ki", "roll_rate_kd", "roll_rate_imax",
        "pitch_rate_kp", "pitch_rate_ki", "pitch_rate_kd", "pitch_rate_imax",
        "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_kd", "yaw_rate_imax",
        "roll_ang_kp", "pitch_ang_kp", "yaw_ang_kp"
    ];

    pidKeys.forEach(key => {
        const slider = document.getElementById(`slider-${key}`);
        const valText = document.getElementById(`val-${key}`);
        if (slider) {
            slider.value = slider.min;
            slider.disabled = true;
        }
        if (valText) {
            valText.value = "";
            valText.disabled = true;
        }
    });

    const monitor = document.getElementById("serial-monitor");
    if (monitor) monitor.textContent = "";
    const output = document.getElementById("cli-output");
    if (output) output.textContent = "";
    const btnCalibrate = document.getElementById("btn-calibrate");
    if (btnCalibrate) {
        btnCalibrate.textContent = "Calibrate";
        btnCalibrate.disabled = false;
        btnCalibrate.removeAttribute("title");
    }
    const btnZero = document.getElementById("btn-zero-biases");
    if (btnZero) {
        btnZero.style.display = "none";
        btnZero.textContent = "Zero Biases";
        btnZero.disabled = true;
    }

    const badgeArm = document.getElementById("badge-arm");
    if (badgeArm) {
        badgeArm.textContent = "DISARMED";
        badgeArm.className = "status-badge badge-disarmed";
    }
    const badgeMode = document.getElementById("badge-mode");
    if (badgeMode) {
        badgeMode.textContent = "UNKNOWN";
        badgeMode.className = "status-badge badge-disarmed";
    }
    const labelVbat = document.getElementById("label-vbat");
    if (labelVbat) labelVbat.textContent = "0.00V";
    const labelLoopTime = document.getElementById("label-loop-time");
    if (labelLoopTime) {
        labelLoopTime.textContent = "0.00ms";
        labelLoopTime.removeAttribute("title");
    }
    
    smoothedTotalTime = 0;
    smoothedHalTime = 0;
    smoothedGncTime = 0;
    smoothedTimeImu = 0;
    smoothedTimeRcrx = 0;
    smoothedTimeMotors = 0;
    smoothedTimeServos = 0;
    smoothedTimeNav = 0;
    smoothedTimeCtl = 0;
    smoothedTimeAlloc = 0;
    smoothedTimeIdle = 0;
    
    const timeFields = ["imu", "rcrx", "nav", "ctl", "alloc", "motors", "servos", "idle"];
    timeFields.forEach(f => {
        const bar = document.getElementById("bar-time-" + f);
        const txt = document.getElementById("txt-time-" + f);
        if (bar) bar.style.width = "0%";
        if (txt) txt.textContent = "0.0 us";
    });
    const txtTotal = document.getElementById("txt-time-total-avg");
    if (txtTotal) txtTotal.textContent = "0.00ms active";
    const batteryInner = document.getElementById("battery-level-inner");
    if (batteryInner) {
        batteryInner.style.width = "0%";
        batteryInner.style.backgroundColor = "#ef4444";
    }
    
    const circles = ["m1", "m2", "m3", "m4", "s1", "s2", "s3", "s4"];
    circles.forEach(id => {
        const el = document.getElementById("circle-" + id);
        const def = id.startsWith("m") ? "0%" : "90°";
        if (el) {
            el.textContent = def;
            el.style.borderColor = "#27272a";
            el.style.boxShadow = "none";
        }
    });

    const imuIds = ["gx", "gy", "gz", "ax", "ay", "az"];
    imuIds.forEach(id => {
        const bar = document.getElementById("imu-bar-" + id);
        const label = document.getElementById("label-val-" + id);
        if (bar) {
            bar.style.height = "0%";
            bar.style.bottom = "50%";
        }
        if (label) {
            label.textContent = id.startsWith("g") ? "0°/s" : "0m/s²";
        }
    });

    const rcChs = ["thr", "rol", "pit", "yaw", "arm", "mod"];
    rcChs.forEach(ch => {
        const bar = document.getElementById("rc-bar-" + ch);
        const label = document.getElementById("label-rc-" + ch);
        if (bar) {
            bar.style.width = "0%";
            if (ch === "rol" || ch === "pit" || ch === "yaw") {
                bar.style.left = "50%";
            } else {
                bar.style.left = "0";
            }
        }
        if (label) label.textContent = "0%";
    });

    incomingBytesBuffer = new Uint8Array(0);
    textAccumulator = "";
}

async function writeRaw(text) {
    if (!writer) return;
    const encoder = new TextEncoder();
    await writer.write(encoder.encode(text));
}

async function readLoop() {
    try {
        while (isConnected && port && port.readable) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value) {
                processIncomingBytes(value);
            }
        }
    } catch (err) {
        console.error(err);
        await disconnect();
    }
}

function processIncomingBytes(newBytes) {
    const temp = new Uint8Array(incomingBytesBuffer.length + newBytes.length);
    temp.set(incomingBytesBuffer);
    temp.set(newBytes, incomingBytesBuffer.length);
    incomingBytesBuffer = temp;

    let scanIdx = 0;
    while (scanIdx < incomingBytesBuffer.length) {
        let foundHeader = false;
        let headerIdx = -1;
        
        for (let i = scanIdx; i <= incomingBytesBuffer.length - 5; i++) {
            if (incomingBytesBuffer[i] === 0xAA && 
                incomingBytesBuffer[i+1] === 0xBB && 
                incomingBytesBuffer[i+2] === 0x01) {
                foundHeader = true;
                headerIdx = i;
                break;
            }
        }

        if (foundHeader) {
            const len = incomingBytesBuffer[headerIdx+3] | (incomingBytesBuffer[headerIdx+4] << 8);
            
            if (len > 0 && len < 1000) {
                const totalPacketLen = 5 + len;
                
                if (headerIdx + totalPacketLen <= incomingBytesBuffer.length) {
                    const payload = incomingBytesBuffer.slice(headerIdx + 5, headerIdx + 5 + len);
                    
                    let parseSuccess = false;
                    try {
                        parseSuccess = parseBinaryALLb(payload);
                    } catch (e) {
                        parseSuccess = false;
                    }
                    
                    if (parseSuccess) {
                        if (headerIdx > 0) {
                            const textBytes = incomingBytesBuffer.slice(0, headerIdx);
                            flushTextBytes(textBytes);
                        }
                        incomingBytesBuffer = incomingBytesBuffer.slice(headerIdx + totalPacketLen);
                        scanIdx = 0;
                        continue;
                    } else {
                        scanIdx = headerIdx + 1;
                        continue;
                    }
                } else {
                    if (len !== ALLb_SIZE + 24) {
                        scanIdx = headerIdx + 1;
                        continue;
                    }
                    if (headerIdx > 0) {
                        const textBytes = incomingBytesBuffer.slice(0, headerIdx);
                        flushTextBytes(textBytes);
                        incomingBytesBuffer = incomingBytesBuffer.slice(headerIdx);
                    }
                    break;
                }
            } else {
                scanIdx = headerIdx + 1;
                continue;
            }
        } else {
            let lastNewlineIdx = -1;
            for (let i = incomingBytesBuffer.length - 1; i >= 0; i--) {
                if (incomingBytesBuffer[i] === 10) {
                    lastNewlineIdx = i;
                    break;
                }
            }

            if (lastNewlineIdx !== -1) {
                const textBytes = incomingBytesBuffer.slice(0, lastNewlineIdx + 1);
                flushTextBytes(textBytes);
                incomingBytesBuffer = incomingBytesBuffer.slice(lastNewlineIdx + 1);
            } else if (incomingBytesBuffer.length > 1000) {
                flushTextBytes(incomingBytesBuffer);
                incomingBytesBuffer = new Uint8Array(0);
            }
            break;
        }
    }
}

function flushTextBytes(bytes) {
    textAccumulator += textDecoder.decode(bytes, { stream: true });
    let lineEndIdx;
    while ((lineEndIdx = textAccumulator.indexOf("\n")) !== -1) {
        const line = textAccumulator.substring(0, lineEndIdx + 1);
        textAccumulator = textAccumulator.substring(lineEndIdx + 1);
        handleIncomingTextLine(line);
    }
}

function handleIncomingTextLine(line) {
    const trimmed = line.trim();
    if (trimmed.startsWith("$")) {
        return;
    }
    const output = document.getElementById("cli-output");
    if (output) {
        output.textContent += line;
        output.scrollTop = output.scrollHeight;
    }
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
    if (placeholder) {
        placeholder.textContent = "Loading parameters from flight controller...";
        placeholder.style.display = "block";
    }

    const pidKeys = [
        "roll_rate_kp", "roll_rate_ki", "roll_rate_kd", "roll_rate_imax",
        "pitch_rate_kp", "pitch_rate_ki", "pitch_rate_kd", "pitch_rate_imax",
        "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_kd", "yaw_rate_imax",
        "roll_ang_kp", "pitch_ang_kp", "yaw_ang_kp"
    ];
    pidKeys.forEach(key => {
        const slider = document.getElementById(`slider-${key}`);
        const valText = document.getElementById(`val-${key}`);
        if (slider) slider.disabled = true;
        if (valText) valText.disabled = true;
    });

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
    const placeholder = document.getElementById("params-loading");
    if (Object.keys(paramsCache).length === 0) {
        if (placeholder) {
            placeholder.textContent = "Failed to load parameters. Try reloading.";
            placeholder.style.display = "block";
        }
        return;
    }
    if (placeholder) {
        placeholder.style.display = "none";
    }

    const pidKeys = [
        "roll_rate_kp", "roll_rate_ki", "roll_rate_kd", "roll_rate_imax",
        "pitch_rate_kp", "pitch_rate_ki", "pitch_rate_kd", "pitch_rate_imax",
        "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_kd", "yaw_rate_imax",
        "roll_ang_kp", "pitch_ang_kp", "yaw_ang_kp"
    ];

    pidKeys.forEach(key => {
        const val = parseFloat(paramsCache[key] || 0);
        const slider = document.getElementById(`slider-${key}`);
        const valText = document.getElementById(`val-${key}`);
        if (slider && valText) {
            slider.value = val;
            slider.disabled = false;
            
            const isKd = key.endsWith("_kd");
            const places = isKd ? 4 : 2;
            valText.value = val.toFixed(places);
            valText.disabled = false;
            
            if (!slider.dataset.listenerAdded) {
                slider.addEventListener("input", (e) => {
                    const currentVal = parseFloat(e.target.value);
                    valText.value = currentVal.toFixed(places);
                    handleParamChange(key, currentVal.toString());
                });
                slider.dataset.listenerAdded = "true";
            }

            if (!valText.dataset.listenerAdded) {
                valText.addEventListener("change", (e) => {
                    let currentVal = parseFloat(e.target.value);
                    if (isNaN(currentVal)) {
                        currentVal = parseFloat(slider.value);
                    }
                    const min = parseFloat(slider.min);
                    const max = parseFloat(slider.max);
                    if (currentVal < min) currentVal = min;
                    if (currentVal > max) currentVal = max;

                    slider.value = currentVal;
                    valText.value = currentVal.toFixed(places);
                    handleParamChange(key, currentVal.toString());
                });
                valText.dataset.listenerAdded = "true";
            }
        }
    });

    const selAlloc = document.getElementById("select-allocator");
    if (selAlloc) {
        if (paramsCache["gnc_allocator"]) {
            selAlloc.value = paramsCache["gnc_allocator"];
        }
        selAlloc.disabled = false;
    }

    checkBiases();
}

function checkBiases() {
    const ax = parseFloat(paramsCache["gnc_nav_imu_calc_accel_bias_x"] || 0);
    const ay = parseFloat(paramsCache["gnc_nav_imu_calc_accel_bias_y"] || 0);
    const az = parseFloat(paramsCache["gnc_nav_imu_calc_accel_bias_z"] || 0);
    const gx = parseFloat(paramsCache["gnc_nav_imu_calc_gyro_bias_x"] || 0);
    const gy = parseFloat(paramsCache["gnc_nav_imu_calc_gyro_bias_y"] || 0);
    const gz = parseFloat(paramsCache["gnc_nav_imu_calc_gyro_bias_z"] || 0);

    const hasBiases = (ax !== 0 || ay !== 0 || az !== 0 || gx !== 0 || gy !== 0 || gz !== 0);
    const btnCalibrate = document.getElementById("btn-calibrate");
    const btnZeroBiases = document.getElementById("btn-zero-biases");

    if (hasBiases) {
        if (btnCalibrate) {
            btnCalibrate.disabled = true;
            btnCalibrate.title = "Biases must be zeroed pre-calibration";
        }
        if (btnZeroBiases) {
            btnZeroBiases.style.display = "inline-block";
            btnZeroBiases.disabled = false;
        }
    } else {
        if (btnCalibrate) {
            btnCalibrate.disabled = false;
            btnCalibrate.removeAttribute("title");
        }
        if (btnZeroBiases) {
            btnZeroBiases.style.display = "none";
            btnZeroBiases.disabled = true;
        }
    }
}

async function zeroBiases() {
    if (!isConnected) return;
    if (!confirm("Zero out all navigation biases and reboot?")) return;
    document.getElementById("btn-zero-biases").disabled = true;
    document.getElementById("btn-zero-biases").textContent = "Zeroing...";
    try {
        await writeRaw("set gnc_nav_imu_calc_accel_bias_x = 0\n");
        await new Promise(r => setTimeout(r, 100));
        await writeRaw("set gnc_nav_imu_calc_accel_bias_y = 0\n");
        await new Promise(r => setTimeout(r, 100));
        await writeRaw("set gnc_nav_imu_calc_accel_bias_z = 0\n");
        await new Promise(r => setTimeout(r, 100));
        await writeRaw("set gnc_nav_imu_calc_gyro_bias_x = 0\n");
        await new Promise(r => setTimeout(r, 100));
        await writeRaw("set gnc_nav_imu_calc_gyro_bias_y = 0\n");
        await new Promise(r => setTimeout(r, 100));
        await writeRaw("set gnc_nav_imu_calc_gyro_bias_z = 0\n");
        await new Promise(r => setTimeout(r, 100));
        await writeRaw("save\n");
        await new Promise(r => setTimeout(r, 200));
        await disconnect();
    } catch (err) {
        alert("Failed to zero biases: " + err.message);
        document.getElementById("btn-zero-biases").textContent = "Zero Biases";
        document.getElementById("btn-zero-biases").disabled = false;
    }
}

function handleParamChange(key, val) {
    if (paramsCache[key] === val) {
        delete modifiedParams[key];
    } else {
        modifiedParams[key] = val;
    }
}

// filterParameters removed

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

async function startCalibration() {
    if (!isConnected) return;
    document.getElementById("btn-calibrate").disabled = true;
    document.getElementById("btn-calibrate").textContent = "Calibrating...";
    calibrationState = "waiting";
    try {
        await writeRaw("calibrate\n");
    } catch (err) {
        alert("Failed to send calibration command: " + err.message);
        document.getElementById("btn-calibrate").textContent = "Calibrate";
        document.getElementById("btn-calibrate").disabled = false;
        calibrationState = "idle";
    }
}

let isTestModeEnabled = false;
let testInterval = null;
let actuatorTestValues = { m1: 0, m2: 0, m3: 0, m4: 0, s1: 90, s2: 90, s3: 90, s4: 90 };

function toggleActuatorTest() {
    if (!isConnected) return;
    
    isTestModeEnabled = !isTestModeEnabled;
    const btn = document.getElementById("btn-toggle-test");
    const status = document.getElementById("test-status");
    const placeholder = document.getElementById("test-placeholder");
    const slidersContainer = document.getElementById("test-sliders");
    
    if (isTestModeEnabled) {
        btn.textContent = "Disable Test Mode";
        btn.className = "btn btn-connected";
        status.textContent = "Test Active";
        status.className = "status-indicator connected";
        placeholder.style.display = "none";
        slidersContainer.style.display = "flex";
        
        buildTestSlidersUI();
        sendActuatorTestData();
        testInterval = setInterval(sendActuatorTestData, 50);
    } else {
        btn.textContent = "Enable Test Mode";
        btn.className = "btn btn-warning";
        status.textContent = "Test Disabled";
        status.className = "status-indicator disconnected";
        placeholder.style.display = "block";
        slidersContainer.style.display = "none";
        
        if (testInterval) {
            clearInterval(testInterval);
            testInterval = null;
        }
        
        const s1_def = parseFloat(paramsCache["ser_default_ang_deg"] || 90).toFixed(2);
        const s2_def = parseFloat(paramsCache["ser_default_ang_deg"] || 90).toFixed(2);
        const s3_def = parseFloat(paramsCache["ser_default_ang_deg"] || 90).toFixed(2);
        const s4_def = parseFloat(paramsCache["ser_default_ang_deg"] || 90).toFixed(2);
        writeRaw(`act_test 0 0.0000 0.0000 0.0000 0.0000 ${s1_def} ${s2_def} ${s3_def} ${s4_def} 0.0\n`);
    }
}

function buildTestSlidersUI() {
    const container = document.getElementById("test-sliders");
    container.innerHTML = "";
    
    const actuators = [
        { id: "m1", name: "Motor 1 (Pin " + paramsCache["mot_m1_pin"] + ")", type: "motor", pinKey: "mot_m1_pin", min: 0, max: 100, unit: "%", def: 0 },
        { id: "m2", name: "Motor 2 (Pin " + paramsCache["mot_m2_pin"] + ")", type: "motor", pinKey: "mot_m2_pin", min: 0, max: 100, unit: "%", def: 0 },
        { id: "m3", name: "Motor 3 (Pin " + paramsCache["mot_m3_pin"] + ")", type: "motor", pinKey: "mot_m3_pin", min: 0, max: 100, unit: "%", def: 0 },
        { id: "m4", name: "Motor 4 (Pin " + paramsCache["mot_m4_pin"] + ")", type: "motor", pinKey: "mot_m4_pin", min: 0, max: 100, unit: "%", def: 0 },
        { id: "s1", name: "Servo 1 (Pin " + paramsCache["ser_s1_pin"] + ")", type: "servo", pinKey: "ser_s1_pin", min: parseFloat(paramsCache["ser_min_ang_deg"] || 0), max: parseFloat(paramsCache["ser_max_ang_deg"] || 180), unit: "°", def: parseFloat(paramsCache["ser_default_ang_deg"] || 90) },
        { id: "s2", name: "Servo 2 (Pin " + paramsCache["ser_s2_pin"] + ")", type: "servo", pinKey: "ser_s2_pin", min: parseFloat(paramsCache["ser_min_ang_deg"] || 0), max: parseFloat(paramsCache["ser_max_ang_deg"] || 180), unit: "°", def: parseFloat(paramsCache["ser_default_ang_deg"] || 90) },
        { id: "s3", name: "Servo 3 (Pin " + paramsCache["ser_s3_pin"] + ")", type: "servo", pinKey: "ser_s3_pin", min: parseFloat(paramsCache["ser_min_ang_deg"] || 0), max: parseFloat(paramsCache["ser_max_ang_deg"] || 180), unit: "°", def: parseFloat(paramsCache["ser_default_ang_deg"] || 90) },
        { id: "s4", name: "Servo 4 (Pin " + paramsCache["ser_s4_pin"] + ")", type: "servo", pinKey: "ser_s4_pin", min: parseFloat(paramsCache["ser_min_ang_deg"] || 0), max: parseFloat(paramsCache["ser_max_ang_deg"] || 180), unit: "°", def: parseFloat(paramsCache["ser_default_ang_deg"] || 90) }
    ];
    
    actuators.forEach(act => {
        const pin = parseInt(paramsCache[act.pinKey] || "255");
        const isDisabled = (pin === 255 || isNaN(pin));
        
        if (actuatorTestValues[act.id] === undefined || !isTestModeEnabled) {
            actuatorTestValues[act.id] = act.def;
        }
        
        const row = document.createElement("div");
        row.className = "test-slider-row";
        if (isDisabled) {
            row.style.opacity = "0.35";
        }
        
        const header = document.createElement("div");
        header.className = "test-slider-header";
        
        const nameSpan = document.createElement("span");
        nameSpan.className = "test-slider-name";
        nameSpan.textContent = act.name;
        header.appendChild(nameSpan);
        
        const valSpan = document.createElement("span");
        valSpan.className = "test-slider-val";
        valSpan.id = "val-" + act.id;
        valSpan.textContent = actuatorTestValues[act.id] + act.unit;
        header.appendChild(valSpan);
        
        row.appendChild(header);
        
        const controlDiv = document.createElement("div");
        controlDiv.className = "test-slider-control";
        
        const btnMinus = document.createElement("button");
        btnMinus.className = "btn btn-small";
        btnMinus.textContent = "-1%";
        btnMinus.disabled = isDisabled || !isTestModeEnabled;
        btnMinus.addEventListener("click", () => {
            adjustSlider(act, -1);
        });
        controlDiv.appendChild(btnMinus);
        
        const slider = document.createElement("input");
        slider.type = "range";
        slider.className = "test-slider-input";
        slider.min = act.min;
        slider.max = act.max;
        slider.step = act.type === "motor" ? 1 : 0.5;
        slider.value = actuatorTestValues[act.id];
        slider.disabled = isDisabled || !isTestModeEnabled;
        slider.id = "slider-" + act.id;
        
        slider.addEventListener("input", (e) => {
            const parsedVal = parseFloat(e.target.value);
            actuatorTestValues[act.id] = parsedVal;
            valSpan.textContent = parsedVal + act.unit;
        });
        controlDiv.appendChild(slider);
        
        const btnPlus = document.createElement("button");
        btnPlus.className = "btn btn-small";
        btnPlus.textContent = "+1%";
        btnPlus.disabled = isDisabled || !isTestModeEnabled;
        btnPlus.addEventListener("click", () => {
            adjustSlider(act, 1);
        });
        controlDiv.appendChild(btnPlus);
        
        row.appendChild(controlDiv);
        container.appendChild(row);
    });
}

function adjustSlider(act, direction) {
    const slider = document.getElementById("slider-" + act.id);
    const valSpan = document.getElementById("val-" + act.id);
    if (!slider || !valSpan) return;
    
    const range = act.max - act.min;
    const step = range * 0.01;
    let val = parseFloat(slider.value) + (direction * step);
    val = Math.max(act.min, Math.min(act.max, val));
    val = Math.round(val * 10) / 10;
    
    slider.value = val;
    actuatorTestValues[act.id] = val;
    valSpan.textContent = val + act.unit;
}

async function sendActuatorTestData() {
    if (!isConnected || !isTestModeEnabled) return;
    
    const m1 = (actuatorTestValues.m1 / 100.0).toFixed(4);
    const m2 = (actuatorTestValues.m2 / 100.0).toFixed(4);
    const m3 = (actuatorTestValues.m3 / 100.0).toFixed(4);
    const m4 = (actuatorTestValues.m4 / 100.0).toFixed(4);
    const s1 = actuatorTestValues.s1.toFixed(2);
    const s2 = actuatorTestValues.s2.toFixed(2);
    const s3 = actuatorTestValues.s3.toFixed(2);
    const s4 = actuatorTestValues.s4.toFixed(2);
    const led = "5.0";
    
    await writeRaw(`act_test 1 ${m1} ${m2} ${m3} ${m4} ${s1} ${s2} ${s3} ${s4} ${led}\n`);
}

function initTabs() {
    const tabButtons = document.querySelectorAll(".tab-btn");
    tabButtons.forEach(btn => {
        btn.addEventListener("click", () => {
            tabButtons.forEach(b => b.classList.remove("active"));
            document.querySelectorAll(".tab-content").forEach(c => c.classList.remove("active"));
            
            btn.classList.add("active");
            const targetId = btn.getAttribute("data-tab");
            const targetContent = document.getElementById(targetId);
            if (targetContent) {
                targetContent.classList.add("active");
            }
        });
    });
}

function calculateFletcher16(data) {
    let sum1 = 0;
    let sum2 = 0;
    for (let i = 0; i < data.length; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

function parseBinaryALLb(flatbufferPayload) {
    try {
        if (flatbufferPayload.byteLength < 8) return false;

        const view = new DataView(flatbufferPayload.buffer, flatbufferPayload.byteOffset, flatbufferPayload.byteLength);

        const rootOffset = view.getInt32(0, true);
        if (rootOffset < 0 || rootOffset >= flatbufferPayload.byteLength) return false;

        const vtableOffset = view.getInt32(rootOffset, true);
        const vtableStart = rootOffset - vtableOffset;
        if (vtableStart < 0 || vtableStart + 6 > flatbufferPayload.byteLength) return false;

        const fieldOffset = view.getUint16(vtableStart + 4, true);
        if (fieldOffset === 0 || rootOffset + fieldOffset + 4 > flatbufferPayload.byteLength) return false;

        const vectorStart = rootOffset + fieldOffset + view.getInt32(rootOffset + fieldOffset, true);
        if (vectorStart < 0 || vectorStart + 4 > flatbufferPayload.byteLength) return false;

        const payloadLength = view.getInt32(vectorStart, true);
        if (payloadLength !== ALLb_SIZE) {
            return false;
        }

        if (vectorStart + 4 + payloadLength > flatbufferPayload.byteLength) return false;
        
        // Now create a DataView for the raw struct inside the FlatBuffer:
        const structView = new DataView(flatbufferPayload.buffer, flatbufferPayload.byteOffset + vectorStart + 4, payloadLength);

        // Helper functions for safe out-of-bounds safe offset reads
        const getFloat32 = (offset) => structView.getFloat32(offset, true);
        const getUint8 = (offset) => structView.getUint8(offset);
        const getUint32 = (offset) => structView.getUint32(offset, true);

        const vbat = getFloat32(ALLb_LAYOUT.vbat);
        const rcArm = getFloat32(ALLb_LAYOUT.rcArm);
        const rcMod = getFloat32(ALLb_LAYOUT.rcMod);
        const rcThr = getFloat32(ALLb_LAYOUT.rcThr);
        const rcRol = getFloat32(ALLb_LAYOUT.rcRol);
        const rcPit = getFloat32(ALLb_LAYOUT.rcPit);
        const rcYaw = getFloat32(ALLb_LAYOUT.rcYaw);
        
        const effRol = getFloat32(ALLb_LAYOUT.effRol);
        const effPit = getFloat32(ALLb_LAYOUT.effPit);
        const effYaw = getFloat32(ALLb_LAYOUT.effYaw);
        
        const armed = getUint8(ALLb_LAYOUT.armed) === 1;
        const mode = getUint32(ALLb_LAYOUT.mode);

        const m1 = getFloat32(ALLb_LAYOUT.m1);
        const m2 = getFloat32(ALLb_LAYOUT.m2);
        const m3 = getFloat32(ALLb_LAYOUT.m3);
        const m4 = getFloat32(ALLb_LAYOUT.m4);
        const s1 = getFloat32(ALLb_LAYOUT.s1);
        const s2 = getFloat32(ALLb_LAYOUT.s2);
        const s3 = getFloat32(ALLb_LAYOUT.s3);
        const s4 = getFloat32(ALLb_LAYOUT.s4);

        const gx = getFloat32(ALLb_LAYOUT.gx) * 57.29577951;
        const gy = getFloat32(ALLb_LAYOUT.gy) * 57.29577951;
        const gz = getFloat32(ALLb_LAYOUT.gz) * 57.29577951;

        const ax = getFloat32(ALLb_LAYOUT.ax);
        const ay = getFloat32(ALLb_LAYOUT.ay);
        const az = getFloat32(ALLb_LAYOUT.az);

        const halTime = getFloat32(ALLb_LAYOUT.halTime);
        const timeImu = getFloat32(ALLb_LAYOUT.timeImu);
        const timeRcrx = getFloat32(ALLb_LAYOUT.timeRcrx);
        const timeMotors = getFloat32(ALLb_LAYOUT.timeMotors);
        const timeServos = getFloat32(ALLb_LAYOUT.timeServos);

        const gncTime = getFloat32(ALLb_LAYOUT.gncTime);
        const timeNav = getFloat32(ALLb_LAYOUT.timeNav);
        const timeCtl = getFloat32(ALLb_LAYOUT.timeCtl);
        const timeAlloc = getFloat32(ALLb_LAYOUT.timeAlloc);

        const cgx = getFloat32(ALLb_LAYOUT.cgx) * 57.29577951;
        const cgy = getFloat32(ALLb_LAYOUT.cgy) * 57.29577951;
        const cgz = getFloat32(ALLb_LAYOUT.cgz) * 57.29577951;

        const isCalibrating = getUint8(ALLb_LAYOUT.isCalibrating) === 1;
        const progress = getFloat32(ALLb_LAYOUT.progress);

    const badgeArm = document.getElementById("badge-arm");
    if (badgeArm) {
        badgeArm.textContent = armed ? "ARMED" : "DISARMED";
        badgeArm.className = armed ? "status-badge badge-armed" : "status-badge badge-disarmed";
    }

    const badgeMode = document.getElementById("badge-mode");
    if (badgeMode) {
        let modeText = "UNKNOWN";
        if (mode === 0) modeText = "RATE";
        else if (mode === 1) modeText = "ANGLE";
        else if (mode === 2) modeText = "ACTUATOR TEST";
        badgeMode.textContent = modeText;
        badgeMode.className = mode === 2 ? "status-badge badge-armed" : "status-badge badge-disarmed";
    }

    const btnCalibrate = document.getElementById("btn-calibrate");
    if (btnCalibrate) {
        if (isCalibrating) {
            calibrationState = "running";
            btnCalibrate.disabled = true;
            btnCalibrate.textContent = `Calibrating (${Math.round(progress * 100)}%)...`;
        } else {
            if (calibrationState === "running") {
                calibrationState = "idle";
                btnCalibrate.textContent = "Calibrate";
                btnCalibrate.disabled = false;
                
                const cliOutput = document.getElementById("cli-output");
                if (cliOutput) {
                    cliOutput.textContent += "\n>>> Calibration complete! Biases saved to config file. Reloading parameters...\n";
                    cliOutput.scrollTop = cliOutput.scrollHeight;
                }
                reloadParams();
            }
        }
    }

    const levelInner = document.getElementById("battery-level-inner");
    const labelVbat = document.getElementById("label-vbat");
    if (levelInner && labelVbat) {
        labelVbat.textContent = vbat.toFixed(2) + "V";
        const frac = Math.max(0.0, Math.min(1.0, (vbat - 21.6) / 3.6));
        levelInner.style.width = (frac * 100) + "%";
        const hue = frac * 120;
        levelInner.style.backgroundColor = `hsl(${hue}, 100%, 45%)`;
    }

    const labelLoopTime = document.getElementById("label-loop-time");
    if (labelLoopTime) {
        const totalTime = halTime + gncTime;
        const alpha = 0.95;
        if (smoothedTotalTime === 0) {
            smoothedTotalTime = totalTime;
            smoothedHalTime = halTime;
            smoothedGncTime = gncTime;
            
            smoothedTimeImu = timeImu;
            smoothedTimeRcrx = timeRcrx;
            smoothedTimeMotors = timeMotors;
            smoothedTimeServos = timeServos;
            smoothedTimeNav = timeNav;
            smoothedTimeCtl = timeCtl;
            smoothedTimeAlloc = timeAlloc;
        } else {
            smoothedTotalTime = smoothedTotalTime * alpha + totalTime * (1 - alpha);
            smoothedHalTime = smoothedHalTime * alpha + halTime * (1 - alpha);
            smoothedGncTime = smoothedGncTime * alpha + gncTime * (1 - alpha);
            
            smoothedTimeImu = smoothedTimeImu * alpha + timeImu * (1 - alpha);
            smoothedTimeRcrx = smoothedTimeRcrx * alpha + timeRcrx * (1 - alpha);
            smoothedTimeMotors = smoothedTimeMotors * alpha + timeMotors * (1 - alpha);
            smoothedTimeServos = smoothedTimeServos * alpha + timeServos * (1 - alpha);
            smoothedTimeNav = smoothedTimeNav * alpha + timeNav * (1 - alpha);
            smoothedTimeCtl = smoothedTimeCtl * alpha + timeCtl * (1 - alpha);
            smoothedTimeAlloc = smoothedTimeAlloc * alpha + timeAlloc * (1 - alpha);
        }

        const looprateHz = parseFloat(paramsCache["looprate_hz"] || "1000");
        const looprateUs = 1000000.0 / looprateHz;
        const activeTimeUs = timeImu + timeRcrx + timeNav + timeCtl + timeAlloc + timeMotors + timeServos;
        const idleTimeUs = Math.max(0, looprateUs - activeTimeUs);

        if (smoothedTimeIdle === 0) {
            smoothedTimeIdle = idleTimeUs;
        } else {
            smoothedTimeIdle = smoothedTimeIdle * alpha + idleTimeUs * (1 - alpha);
        }

        const now = performance.now();
        if (now - lastLoopTimeUpdate > 250) {
            labelLoopTime.textContent = smoothedTotalTime.toFixed(2) + "ms";
            labelLoopTime.title = `HAL: ${smoothedHalTime.toFixed(2)}ms, GNC: ${smoothedGncTime.toFixed(2)}ms (Average)`;
            
            // Update detailed timing texts
            const setTimingText = (id, valUs) => {
                const el = document.getElementById(id);
                if (el) el.textContent = valUs.toFixed(1) + " us";
            };
            setTimingText("txt-time-imu", smoothedTimeImu);
            setTimingText("txt-time-rcrx", smoothedTimeRcrx);
            setTimingText("txt-time-nav", smoothedTimeNav);
            setTimingText("txt-time-ctl", smoothedTimeCtl);
            setTimingText("txt-time-alloc", smoothedTimeAlloc);
            setTimingText("txt-time-motors", smoothedTimeMotors);
            setTimingText("txt-time-servos", smoothedTimeServos);
            setTimingText("txt-time-idle", smoothedTimeIdle);
            
            const txtTotal = document.getElementById("txt-time-total-avg");
            if (txtTotal) {
                const activeMs = (smoothedTimeImu + smoothedTimeRcrx + smoothedTimeNav + smoothedTimeCtl + smoothedTimeAlloc + smoothedTimeMotors + smoothedTimeServos) / 1000.0;
                txtTotal.textContent = activeMs.toFixed(2) + "ms active";
            }

            // Update stacked budget bar widths
            const totalBudgetUs = Math.max(looprateUs, smoothedTimeImu + smoothedTimeRcrx + smoothedTimeNav + smoothedTimeCtl + smoothedTimeAlloc + smoothedTimeMotors + smoothedTimeServos + smoothedTimeIdle);
            const setBarWidth = (id, valUs) => {
                const el = document.getElementById(id);
                if (el) {
                    const pct = (valUs / totalBudgetUs) * 100;
                    el.style.width = pct.toFixed(2) + "%";
                }
            };
            setBarWidth("bar-time-imu", smoothedTimeImu);
            setBarWidth("bar-time-rcrx", smoothedTimeRcrx);
            setBarWidth("bar-time-nav", smoothedTimeNav);
            setBarWidth("bar-time-ctl", smoothedTimeCtl);
            setBarWidth("bar-time-alloc", smoothedTimeAlloc);
            setBarWidth("bar-time-motors", smoothedTimeMotors);
            setBarWidth("bar-time-servos", smoothedTimeServos);
            setBarWidth("bar-time-idle", smoothedTimeIdle);

            lastLoopTimeUpdate = now;
        }
    }

    function updateCircle(id, val, isFraction) {
        const el = document.getElementById("circle-" + id);
        const box = document.getElementById("circle-box-" + id);
        if (!el || !box) return;
        
        const pinKey = id.startsWith("m") ? "mot_" + id + "_pin" : "ser_" + id + "_pin";
        const pin = parseInt(paramsCache[pinKey] || "255");
        if (pin === 255 || isNaN(pin)) {
            box.style.display = "none";
            return;
        }
        box.style.display = "flex";

        let pct = 0;
        if (isFraction) {
            pct = val;
            el.textContent = Math.round(pct * 100) + "%";
        } else {
            const minDeg = parseFloat(paramsCache["ser_min_ang_deg"] || 0);
            const maxDeg = parseFloat(paramsCache["ser_max_ang_deg"] || 180);
            const range = maxDeg - minDeg;
            pct = range > 0 ? (val - minDeg) / range : 0.5;
            el.textContent = Math.round(val) + "°";
        }
        
        pct = Math.max(0.0, Math.min(1.0, pct));
        const hue = (1.0 - pct) * 120;
        el.style.borderColor = `hsl(${hue}, 100%, 45%)`;
        el.style.boxShadow = `0 0 8px hsl(${hue}, 100%, 45%, 0.3)`;
    }

    updateCircle("m1", m1, true);
    updateCircle("m2", m2, true);
    updateCircle("m3", m3, true);
    updateCircle("m4", m4, true);
    updateCircle("s1", s1, false);
    updateCircle("s2", s2, false);
    updateCircle("s3", s3, false);
    updateCircle("s4", s4, false);

    function updateImuBar(type, axis, val) {
        const id = type === "gyro" ? "g" + axis : "a" + axis;
        const bar = document.getElementById("imu-bar-" + id);
        const label = document.getElementById("label-val-" + id);
        if (!bar || !label) return;

        const maxScale = type === "gyro" ? 500.0 : 20.0;
        const unit = type === "gyro" ? "°/s" : "m/s²";
        const clampedVal = Math.max(-maxScale, Math.min(maxScale, val));
        
        const roundedVal = type === "gyro" ? Math.round(val) : parseFloat(val.toFixed(1));
        const sign = roundedVal > 0 ? "+" : "";
        label.textContent = sign + roundedVal + unit;

        const pct = (clampedVal / maxScale) * 50;

        if (pct >= 0) {
            bar.style.height = pct + "%";
            bar.style.bottom = "50%";
        } else {
            bar.style.height = Math.abs(pct) + "%";
            bar.style.bottom = (50 - Math.abs(pct)) + "%";
        }
    }

    updateImuBar("gyro", "x", gx);
    updateImuBar("gyro", "y", gy);
    updateImuBar("gyro", "z", gz);
    updateImuBar("accel", "x", ax);
    updateImuBar("accel", "y", ay);
    updateImuBar("accel", "z", az);

    function updateRcBar(ch, val) {
        const bar = document.getElementById("rc-bar-" + ch);
        const label = document.getElementById("label-rc-" + ch);
        if (!bar || !label) return;

        if (ch === "rol" || ch === "pit" || ch === "yaw") {
            const clampedVal = Math.max(-1.0, Math.min(1.0, val));
            const pctVal = Math.round(clampedVal * 100);
            const sign = pctVal > 0 ? "+" : "";
            label.textContent = sign + pctVal + "%";

            const pct = clampedVal * 50;
            if (pct >= 0) {
                bar.style.width = pct + "%";
                bar.style.left = "50%";
            } else {
                bar.style.width = Math.abs(pct) + "%";
                bar.style.left = (50 - Math.abs(pct)) + "%";
            }
        } else {
            const pct = Math.max(0.0, Math.min(1.0, val));
            bar.style.width = (pct * 100) + "%";
            bar.style.left = "0";
            label.textContent = Math.round(pct * 100) + "%";
        }
    }

    updateRcBar("thr", rcThr);
    updateRcBar("rol", rcRol);
    updateRcBar("pit", rcPit);
    updateRcBar("yaw", rcYaw);
    updateRcBar("arm", rcArm);
    updateRcBar("mod", rcMod);

    // Update Control Rates Telemetry labels
    const setRateText = (id, val) => {
        const el = document.getElementById(id);
        if (el) el.textContent = Math.round(val);
    };
    setRateText("txt-rate-cmd-x", cgx);
    setRateText("txt-rate-meas-x", gx);
    setRateText("txt-rate-cmd-y", cgy);
    setRateText("txt-rate-meas-y", gy);
    setRateText("txt-rate-cmd-z", cgz);
    setRateText("txt-rate-meas-z", gz);

    if (chartInstance) {
        chartInstance.addData(cgx, gx, cgy, gy, cgz, gz);
    }

    // Update Allocate Tab Motor Graphics
    const updateMotorIndicator = (id, val) => {
        const elText = document.getElementById(`val-alloc-${id}`);
        const elCircle = document.getElementById(`circle-alloc-${id}`);
        if (elText) elText.textContent = val.toFixed(6);
        if (elCircle) {
            const opacity = 0.1 + 0.8 * Math.max(0.0, Math.min(1.0, val));
            elCircle.style.fillOpacity = opacity;
        }
    };
    updateMotorIndicator("m1", m1);
    updateMotorIndicator("m2", m2);
    updateMotorIndicator("m3", m3);
    updateMotorIndicator("m4", m4);
 
    // Update Axis Rotation Efforts
    const updateEffortBar = (axis, val) => {
        const elText = document.getElementById(`txt-effort-${axis}`);
        const elBar = document.getElementById(`bar-effort-${axis}`);
        if (elText) {
            const sign = val >= 0 ? "+" : "";
            elText.textContent = sign + val.toFixed(4);
        }
        if (elBar) {
            const clamped = Math.max(-1.0, Math.min(1.0, val));
            const percentWidth = Math.abs(clamped) * 50; // 0% to 50%
            elBar.style.width = percentWidth.toFixed(2) + "%";
            if (clamped >= 0) {
                elBar.style.left = "50%";
            } else {
                elBar.style.left = (50 - percentWidth).toFixed(2) + "%";
            }
        }
    };
    updateEffortBar("roll", effRol);
    updateEffortBar("pitch", effPit);
    updateEffortBar("yaw", effYaw);
 
    return true;
    } catch (e) {
        console.error("Error parsing ALLb packet: ", e);
        return false;
    }
}

class TelemetryChart {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.historySize = 150;
        this.data = {
            rollSetpoint: new Array(this.historySize).fill(0),
            rollMeasured: new Array(this.historySize).fill(0),
            pitchSetpoint: new Array(this.historySize).fill(0),
            pitchMeasured: new Array(this.historySize).fill(0),
            yawSetpoint: new Array(this.historySize).fill(0),
            yawMeasured: new Array(this.historySize).fill(0)
        };
    }
    
    addData(rsp, rmeas, psp, pmeas, ysp, ymeas) {
        this.data.rollSetpoint.push(rsp);
        this.data.rollSetpoint.shift();
        this.data.rollMeasured.push(rmeas);
        this.data.rollMeasured.shift();
        this.data.pitchSetpoint.push(psp);
        this.data.pitchSetpoint.shift();
        this.data.pitchMeasured.push(pmeas);
        this.data.pitchMeasured.shift();
        this.data.yawSetpoint.push(ysp);
        this.data.yawSetpoint.shift();
        this.data.yawMeasured.push(ymeas);
        this.data.yawMeasured.shift();
        this.draw();
    }
    
    draw() {
        const dpr = window.devicePixelRatio || 1;
        const rect = this.canvas.getBoundingClientRect();
        
        if (this.canvas.width !== rect.width * dpr || this.canvas.height !== rect.height * dpr) {
            this.canvas.width = rect.width * dpr;
            this.canvas.height = rect.height * dpr;
            this.ctx.scale(dpr, dpr);
        }
        
        const width = rect.width;
        const height = rect.height;
        this.ctx.clearRect(0, 0, width, height);
        
        // Draw grid lines
        this.ctx.strokeStyle = '#1f1f23';
        this.ctx.lineWidth = 1;
        this.ctx.beginPath();
        for (let i = 1; i < 4; i++) {
            const y = (height / 4) * i;
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(width, y);
        }
        this.ctx.stroke();
        
        // Find max absolute value to scale
        let maxVal = 50;
        const allArrays = Object.values(this.data);
        for (const arr of allArrays) {
            for (const val of arr) {
                if (Math.abs(val) > maxVal) {
                    maxVal = Math.abs(val);
                }
            }
        }
        maxVal = Math.ceil(maxVal / 10) * 10;
        
        // Draw zero line
        const zeroY = height / 2;
        this.ctx.strokeStyle = '#27272a';
        this.ctx.lineWidth = 1.5;
        this.ctx.beginPath();
        this.ctx.moveTo(0, zeroY);
        this.ctx.lineTo(width, zeroY);
        this.ctx.stroke();
        
        const drawLine = (dataArray, color, isDashed = false) => {
            this.ctx.strokeStyle = color;
            this.ctx.lineWidth = isDashed ? 1.5 : 2;
            if (isDashed) {
                this.ctx.setLineDash([4, 4]);
            } else {
                this.ctx.setLineDash([]);
            }
            this.ctx.beginPath();
            for (let i = 0; i < this.historySize; i++) {
                const x = (width / (this.historySize - 1)) * i;
                const y = zeroY - (dataArray[i] / maxVal) * (zeroY * 0.9);
                if (i === 0) {
                    this.ctx.moveTo(x, y);
                } else {
                    this.ctx.lineTo(x, y);
                }
            }
            this.ctx.stroke();
        };
        
        drawLine(this.data.rollSetpoint, '#f87171', true);
        drawLine(this.data.rollMeasured, '#ef4444', false);
        drawLine(this.data.pitchSetpoint, '#60a5fa', true);
        drawLine(this.data.pitchMeasured, '#3b82f6', false);
        drawLine(this.data.yawSetpoint, '#34d399', true);
        drawLine(this.data.yawMeasured, '#10b981', false);
        
        this.ctx.fillStyle = '#a1a1aa';
        this.ctx.font = '10px sans-serif';
        this.ctx.fillText(`+${maxVal}°/s`, 8, 14);
        this.ctx.fillText(`-${maxVal}°/s`, 8, height - 8);
        this.ctx.fillText('0°/s', 8, zeroY - 4);
    }
}
