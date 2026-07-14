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
    document.getElementById("param-search").addEventListener("input", filterParameters);
    document.getElementById("cli-input").addEventListener("keypress", (e) => {
        if (e.key === "Enter") sendCliCommand();
    });
    document.getElementById("btn-send-cli").addEventListener("click", sendCliCommand);
    initTabs();
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
    const controls = ["btn-refresh", "btn-save", "btn-reboot", "btn-defaults", "btn-zero-biases", "btn-calibrate", "cli-input", "btn-send-cli", "param-search", "btn-toggle-test"];
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

    const axes = ["x", "y", "z"];
    axes.forEach(axis => {
        const bar = document.getElementById("gyro-bar-" + axis);
        const label = document.getElementById("label-rate-" + axis);
        if (bar) {
            bar.style.width = "0%";
            bar.style.left = "50%";
        }
        if (label) {
            label.textContent = "0%";
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
                    
                    if (headerIdx > 0) {
                        const textBytes = incomingBytesBuffer.slice(0, headerIdx);
                        flushTextBytes(textBytes);
                    }
                    
                    parseBinaryALLb(payload);
                    
                    incomingBytesBuffer = incomingBytesBuffer.slice(headerIdx + totalPacketLen);
                    scanIdx = 0;
                    continue;
                } else {
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

async function startCalibration() {
    if (!isConnected) return;
    document.getElementById("btn-calibrate").disabled = true;
    document.getElementById("btn-calibrate").textContent = "Calibrating...";
    try {
        await writeRaw("calibrate\n");
    } catch (err) {
        alert("Failed to send calibration command: " + err.message);
        document.getElementById("btn-calibrate").textContent = "Calibrate";
        document.getElementById("btn-calibrate").disabled = false;
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
    const view = new DataView(flatbufferPayload.buffer, flatbufferPayload.byteOffset, flatbufferPayload.byteLength);

    // Extract vector of bytes from the FlatBuffer:
    const rootOffset = view.getInt32(0, true);
    const vtableOffset = view.getInt32(rootOffset, true);
    const vtableStart = rootOffset - vtableOffset;
    const fieldOffset = view.getUint16(vtableStart + 4, true);
    if (fieldOffset === 0) {
        console.error("FlatBuffer payload field not found");
        return;
    }
    const vectorStart = rootOffset + fieldOffset + view.getInt32(rootOffset + fieldOffset, true);
    const payloadLength = view.getInt32(vectorStart, true);
    
    // Now create a DataView for the raw struct inside the FlatBuffer:
    const structView = new DataView(flatbufferPayload.buffer, flatbufferPayload.byteOffset + vectorStart + 4, payloadLength);

    const vbat = structView.getFloat32(48, true);

    const rcArm = structView.getFloat32(24, true);
    const rcMod = structView.getFloat32(28, true);
    const rcThr = structView.getFloat32(32, true);
    const rcRol = structView.getFloat32(36, true);
    const rcPit = structView.getFloat32(40, true);
    const rcYaw = structView.getFloat32(44, true);

    const armed = structView.getUint8(64) === 1;
    const mode = structView.getUint32(68, true);

    const m1 = structView.getFloat32(72, true);
    const m2 = structView.getFloat32(76, true);
    const m3 = structView.getFloat32(80, true);
    const m4 = structView.getFloat32(84, true);
    const s1 = structView.getFloat32(88, true);
    const s2 = structView.getFloat32(92, true);
    const s3 = structView.getFloat32(96, true);
    const s4 = structView.getFloat32(100, true);

    const gx = structView.getFloat32(112, true) * 57.29577951;
    const gy = structView.getFloat32(116, true) * 57.29577951;
    const gz = structView.getFloat32(120, true) * 57.29577951;

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

    const levelInner = document.getElementById("battery-level-inner");
    const labelVbat = document.getElementById("label-vbat");
    if (levelInner && labelVbat) {
        labelVbat.textContent = vbat.toFixed(2) + "V";
        const frac = Math.max(0.0, Math.min(1.0, (vbat - 21.6) / 3.6));
        levelInner.style.width = (frac * 100) + "%";
        const hue = frac * 120;
        levelInner.style.backgroundColor = `hsl(${hue}, 100%, 45%)`;
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

    function updateGyroBar(axis, rateVal) {
        const bar = document.getElementById("gyro-bar-" + axis);
        const label = document.getElementById("label-rate-" + axis);
        if (!bar || !label) return;

        const maxRate = 500.0;
        const clampedVal = Math.max(-maxRate, Math.min(maxRate, rateVal));
        
        const pctVal = Math.round((clampedVal / maxRate) * 100);
        const sign = pctVal > 0 ? "+" : "";
        label.textContent = sign + pctVal + "%";

        const pct = (clampedVal / maxRate) * 50;

        if (pct >= 0) {
            bar.style.width = pct + "%";
            bar.style.left = "50%";
        } else {
            bar.style.width = Math.abs(pct) + "%";
            bar.style.left = (50 - Math.abs(pct)) + "%";
        }
    }

    updateGyroBar("x", gx);
    updateGyroBar("y", gy);
    updateGyroBar("z", gz);

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
}
