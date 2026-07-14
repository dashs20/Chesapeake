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
    const controls = ["btn-refresh", "btn-save", "btn-reboot", "btn-defaults", "btn-calibrate", "cli-input", "btn-send-cli", "param-search", "btn-toggle-test"];
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
            label.textContent = "0.0°/s";
        }
    });

    const rcChs = ["thr", "rol", "pit", "yaw", "arm", "mod"];
    rcChs.forEach(ch => {
        const bar = document.getElementById("rc-bar-" + ch);
        const label = document.getElementById("label-rc-" + ch);
        if (bar) bar.style.width = "0%";
        if (label) label.textContent = "0%";
    });
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
            parseTelemetryLine(trimmed);
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

function parseTelemetryLine(line) {
    const parts = line.split(",");
    const data = {};
    for (let i = 1; i < parts.length; i += 2) {
        if (i + 1 < parts.length) {
            data[parts[i]] = parts[i+1];
        }
    }

    const badgeArm = document.getElementById("badge-arm");
    if (badgeArm && data.ARMED !== undefined) {
        const isArmed = data.ARMED === "1";
        badgeArm.textContent = isArmed ? "ARMED" : "DISARMED";
        badgeArm.className = isArmed ? "status-badge badge-armed" : "status-badge badge-disarmed";
    }

    const badgeMode = document.getElementById("badge-mode");
    if (badgeMode && data.MODE !== undefined) {
        let modeText = "UNKNOWN";
        if (data.MODE === "0") modeText = "RATE";
        else if (data.MODE === "1") modeText = "ANGLE";
        else if (data.MODE === "2") modeText = "ACTUATOR TEST";
        badgeMode.textContent = modeText;
        badgeMode.className = data.MODE === "2" ? "status-badge badge-armed" : "status-badge badge-disarmed";
    }

    const levelInner = document.getElementById("battery-level-inner");
    const labelVbat = document.getElementById("label-vbat");
    if (levelInner && labelVbat && data.VBAT !== undefined) {
        const vbat = parseFloat(data.VBAT);
        labelVbat.textContent = vbat.toFixed(2) + "V";
        
        const frac = Math.max(0.0, Math.min(1.0, (vbat - 21.6) / 3.6));
        levelInner.style.width = (frac * 100) + "%";
        
        const hue = frac * 120;
        levelInner.style.backgroundColor = `hsl(${hue}, 100%, 45%)`;
    }

    function updateCircle(id, value, isFraction) {
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

        const val = parseFloat(value);
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

    if (data.M1 !== undefined) updateCircle("m1", data.M1, true);
    if (data.M2 !== undefined) updateCircle("m2", data.M2, true);
    if (data.M3 !== undefined) updateCircle("m3", data.M3, true);
    if (data.M4 !== undefined) updateCircle("m4", data.M4, true);

    if (data.S1 !== undefined) updateCircle("s1", data.S1, false);
    if (data.S2 !== undefined) updateCircle("s2", data.S2, false);
    if (data.S3 !== undefined) updateCircle("s3", data.S3, false);
    if (data.S4 !== undefined) updateCircle("s4", data.S4, false);

    function updateGyroBar(axis, valStr) {
        const bar = document.getElementById("gyro-bar-" + axis);
        const label = document.getElementById("label-rate-" + axis);
        if (!bar || !label) return;

        const val = parseFloat(valStr);
        label.textContent = val.toFixed(1) + "°/s";

        const maxRate = 500.0;
        const clampedVal = Math.max(-maxRate, Math.min(maxRate, val));
        const pct = (clampedVal / maxRate) * 50;

        if (pct >= 0) {
            bar.style.width = pct + "%";
            bar.style.left = "50%";
        } else {
            bar.style.width = Math.abs(pct) + "%";
            bar.style.left = (50 - Math.abs(pct)) + "%";
        }
    }

    function updateRcBar(ch, valStr) {
        const bar = document.getElementById("rc-bar-" + ch);
        const label = document.getElementById("label-rc-" + ch);
        if (!bar || !label) return;

        const val = parseFloat(valStr);
        const pct = Math.max(0.0, Math.min(1.0, val));
        bar.style.width = (pct * 100) + "%";
        label.textContent = Math.round(pct * 100) + "%";
    }

    if (data.GX !== undefined) updateGyroBar("x", data.GX);
    if (data.GY !== undefined) updateGyroBar("y", data.GY);
    if (data.GZ !== undefined) updateGyroBar("z", data.GZ);

    if (data.RC_THR !== undefined) updateRcBar("thr", data.RC_THR);
    if (data.RC_ROL !== undefined) updateRcBar("rol", data.RC_ROL);
    if (data.RC_PIT !== undefined) updateRcBar("pit", data.RC_PIT);
    if (data.RC_YAW !== undefined) updateRcBar("yaw", data.RC_YAW);
    if (data.RC_ARM !== undefined) updateRcBar("arm", data.RC_ARM);
    if (data.RC_MOD !== undefined) updateRcBar("mod", data.RC_MOD);
}
