let port = null;
let reader = null;
let writer = null;
let isConnected = false;
let paramsCache = {};
let modifiedParams = {};
let serialBuffer = "";
let pendingCommandType = null; // 'dump', 'save', 'defaults', 'reboot'

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
    document.getElementById("btn-connect").addEventListener("click", toggleConnection);
    document.getElementById("btn-refresh").addEventListener("click", reloadParams);
    document.getElementById("btn-save").addEventListener("click", saveParamsToBoard);
    document.getElementById("btn-reboot").addEventListener("click", rebootBoard);
    document.getElementById("btn-defaults").addEventListener("click", resetToDefaults);
    document.getElementById("param-search").addEventListener("input", filterParameters);
    document.getElementById("cli-input").addEventListener("keypress", handleCliKeyPress);
    document.getElementById("btn-send-cli").addEventListener("click", sendCliCommand);
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
    if (isConnected) {
        await disconnect();
    } else {
        await connect();
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
        readLoop();

        setTimeout(async () => {
            await writeRaw("\n");
            await reloadParams();
        }, 500);

    } catch (err) {
        console.error("Connection failed:", err);
        alert("Failed to connect: " + err.message);
        updateConnectionUI(false);
    }
}

async function disconnect() {
    isConnected = false;
    
    if (reader) {
        try {
            await reader.cancel();
        } catch (e) {}
        reader.releaseLock();
        reader = null;
    }
    
    if (writer) {
        writer.releaseLock();
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
    const controls = ["btn-refresh", "btn-save", "btn-reboot", "btn-defaults", "cli-input", "btn-send-cli"];

    if (connected) {
        btn.textContent = "Disconnect";
        btn.classList.add("connected");
        status.textContent = "Connected";
        status.className = "status-indicator connected";
        controls.forEach(id => document.getElementById(id).disabled = false);
    } else {
        btn.textContent = "Connect";
        btn.classList.remove("connected");
        status.textContent = "Disconnected";
        status.className = "status-indicator disconnected";
        controls.forEach(id => document.getElementById(id).disabled = true);
    }
}

function clearBoardUI() {
    paramsCache = {};
    modifiedParams = {};
    document.getElementById("params-loading").style.display = "block";
    document.getElementById("params-grid").style.display = "none";
    document.getElementById("params-grid").innerHTML = "";
    
    const summaryIds = ["sum-m1", "sum-m2", "sum-m3", "sum-m4", "sum-looprate", "sum-attitude"];
    summaryIds.forEach(id => document.getElementById(id).textContent = "-");
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
        disconnect();
    }
}

function handleIncomingChunk(chunk) {
    serialBuffer += chunk;
    appendCliOutput(chunk);
    
    let lineEndIdx;
    while ((lineEndIdx = serialBuffer.indexOf("\n")) !== -1) {
        const line = serialBuffer.substring(0, lineEndIdx).trim();
        serialBuffer = serialBuffer.substring(lineEndIdx + 1);
        
        if (pendingCommandType === "dump") {
            parseDumpLine(line);
        }
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
    
    document.getElementById("params-loading").textContent = "Loading parameters from flight controller...";
    document.getElementById("params-loading").style.display = "block";
    document.getElementById("params-grid").style.display = "none";

    await writeRaw("dump\n");

    setTimeout(() => {
        pendingCommandType = null;
        buildParametersUI();
        updateSummaryUI();
    }, 1500);
}

function buildParametersUI() {
    const grid = document.getElementById("params-grid");
    grid.innerHTML = "";
    
    if (Object.keys(paramsCache).length === 0) {
        document.getElementById("params-loading").textContent = "Failed to load parameters. Try refreshing.";
        return;
    }
    
    document.getElementById("params-loading").style.display = "none";
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
    document.getElementById("sum-m1").textContent = paramsCache["mot_m1_pin"] || "-";
    document.getElementById("sum-m2").textContent = paramsCache["mot_m2_pin"] || "-";
    document.getElementById("sum-m3").textContent = paramsCache["mot_m3_pin"] || "-";
    document.getElementById("sum-m4").textContent = paramsCache["mot_m4_pin"] || "-";
    document.getElementById("sum-looprate").textContent = paramsCache["gnc_looprate_hz"] ? `${paramsCache["gnc_looprate_hz"]} Hz` : "-";
    document.getElementById("sum-attitude").textContent = paramsCache["angle_loop_hz"] ? `${paramsCache["angle_loop_hz"]} Hz` : "-";
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
