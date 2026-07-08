/**
 * Chesapeake FSW Configurator - Application Logic
 * Integrates Web Serial communication, UI syncing, and a 3D Three.js rendering engine.
 * Theme: Maryland Calvert/Crossland Flag aesthetic (Black, Gold, Red, White)
 */

// Serial Connection State
let port = null;
let reader = null;
let writer = null;
let keepReading = false;
let readPromise = null;
let serialBuffer = '';

// Active values tracker (maps param names to numeric values)
const activeConfig = {};

// Three.js Variables
let scene, camera, renderer, vehicleMesh;

// DOM Elements
const btnConnect = document.getElementById('btn-connect');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const btnLoad = document.getElementById('btn-load');
const btnSave = document.getElementById('btn-save');
const btnDefaults = document.getElementById('btn-defaults');
const terminalOutput = document.getElementById('terminal-output');
const terminalInput = document.getElementById('terminal-input');

// Initialize App
window.addEventListener('DOMContentLoaded', () => {
  setupUIHandlers();
  init3DVisualizer();
});

// Setup range sliders <-> numeric inputs sync
function setupUIHandlers() {
  const sliders = document.querySelectorAll('input[type="range"]');
  sliders.forEach(slider => {
    const id = slider.id.replace('slider-', '');
    const numInput = document.getElementById(`num-${id}`);
    
    if (numInput) {
      // Sync slider drag to numeric field
      slider.addEventListener('input', (e) => {
        numInput.value = e.target.value;
        activeConfig[id] = parseFloat(e.target.value);
        if (id.startsWith('imu_euler_')) {
          update3DRotation();
        }
      });
      
      // Sync numeric field change back to slider
      numInput.addEventListener('change', (e) => {
        let val = parseFloat(e.target.value);
        const min = parseFloat(slider.min);
        const max = parseFloat(slider.max);
        
        // Clamp bounds
        if (val < min) val = min;
        if (val > max) val = max;
        
        // Snap to nearest 45 for IMU euler inputs
        if (id.startsWith('imu_euler_')) {
          val = Math.round(val / 45) * 45;
        }
        
        e.target.value = val;
        slider.value = val;
        activeConfig[id] = val;
        if (id.startsWith('imu_euler_')) {
          update3DRotation();
        }
      });
    }
  });

  // Allocator select changes
  const selectAlloc = document.getElementById('select-allocator_type');
  selectAlloc.addEventListener('change', (e) => {
    activeConfig['allocator_type'] = parseFloat(e.target.value);
  });

  // Standard inputs without sliders
  const directInputs = ['looprate_hz', 'max_rate_degps', 'imu_lpf_fc_hz', 'servo1_offset_deg', 'servo2_offset_deg', 'theta_min_deg', 'theta_max_deg'];
  directInputs.forEach(id => {
    const input = document.getElementById(`num-${id}`);
    if (input) {
      input.addEventListener('change', (e) => {
        activeConfig[id] = parseFloat(e.target.value);
      });
    }
  });

  // Action Buttons Listeners
  btnConnect.addEventListener('click', toggleConnection);
  btnLoad.addEventListener('click', () => sendCommand('dump'));
  btnSave.addEventListener('click', saveConfigToBoard);
  btnDefaults.addEventListener('click', () => sendCommand('defaults'));

  // Terminal Input submit handler
  terminalInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
      const command = terminalInput.value.trim();
      if (command) {
        sendCommand(command);
        terminalInput.value = '';
      }
    }
  });
}

// ------------------------------
// Web Serial Communication
// ------------------------------

async function toggleConnection() {
  if (port) {
    // Disconnect
    logTerminal('Disconnecting...', 'recv');
    keepReading = false;
    if (reader) {
      try {
        await reader.cancel();
      } catch (err) {
        console.error('Error canceling reader:', err);
      }
    }
    if (readPromise) {
      await readPromise;
    }
    
    try {
      if (writer) await writer.close();
      if (port) await port.close();
    } catch (err) {
      console.error('Error closing port:', err);
    }
    
    port = null;
    reader = null;
    writer = null;
    setConnectionState(false);
    logTerminal('Device disconnected.', 'error');
  } else {
    // Connect
    if (!('serial' in navigator)) {
      alert('Web Serial is not supported in this browser. Please use Google Chrome or Microsoft Edge.');
      return;
    }
    
    try {
      port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });
      
      writer = port.writable.getWriter();
      setConnectionState(true);
      logTerminal('Device connected successfully!', 'sent');
      
      // Start reading loop
      keepReading = true;
      readPromise = readSerialLoop();
      
      // Query parameters
      setTimeout(() => {
        sendCommand('dump');
      }, 500);
      
    } catch (err) {
      console.error('Serial connection failed:', err);
      logTerminal(`Connection failed: ${err.message}`, 'error');
      port = null;
      setConnectionState(false);
    }
  }
}

async function readSerialLoop() {
  while (port && keepReading) {
    try {
      reader = port.readable.getReader();
      while (keepReading) {
        const { value, done } = await reader.read();
        if (done) {
          break;
        }
        if (value) {
          const text = new TextDecoder().decode(value);
          processIncomingData(text);
        }
      }
    } catch (err) {
      console.error('Serial read error:', err);
      logTerminal(`Read Error: ${err.message}`, 'error');
      break;
    } finally {
      if (reader) {
        reader.releaseLock();
        reader = null;
      }
    }
  }
}

function processIncomingData(text) {
  serialBuffer += text;
  
  let lineEnd = serialBuffer.indexOf('\n');
  while (lineEnd !== -1) {
    const line = serialBuffer.substring(0, lineEnd).trim();
    serialBuffer = serialBuffer.substring(lineEnd + 1);
    
    if (line) {
      logTerminal(line, 'recv');
      parseDumpLine(line);
    }
    lineEnd = serialBuffer.indexOf('\n');
  }
}

// Parse dump lines: "set pid_x_kp = 0.002200" or "set pin_elrs_rx = 7"
function parseDumpLine(line) {
  if (line.toLowerCase().startsWith('set ')) {
    // strip "set "
    let clean = line.substring(4).trim();
    // split by '='
    let parts = clean.split('=');
    if (parts.length === 2) {
      const rawName = parts[0].trim();
      const rawVal = parseFloat(parts[1].trim());
      
      if (!isNaN(rawVal)) {
        // remove "pin_" prefix if it exists to match activeConfig key
        let name = rawName;
        let isPin = false;
        if (rawName.startsWith('pin_')) {
          name = rawName.substring(4);
          isPin = true;
        }
        
        activeConfig[name] = rawVal;
        updateUIField(name, rawVal, isPin);
      }
    }
  }
}

function updateUIField(name, value, isPin) {
  // Update Range Sliders
  const prefix = isPin ? 'pin_' : '';
  const slider = document.getElementById(`slider-${prefix}${name}`);
  const numInput = document.getElementById(`num-${prefix}${name}`);
  
  if (slider) slider.value = value;
  if (numInput) numInput.value = value;

  // Handles dropdown selects
  if (name === 'allocator_type') {
    const select = document.getElementById('select-allocator_type');
    if (select) select.value = Math.round(value);
  }

  // Update 3D visualizer rotation if Euler angles loaded
  if (name.startsWith('imu_euler_')) {
    update3DRotation();
  }
}

async function sendCommand(commandText) {
  if (!writer) return;
  
  logTerminal(`> ${commandText}`, 'sent');
  try {
    const encoder = new TextEncoder();
    const data = encoder.encode(commandText + '\n');
    await writer.write(data);
  } catch (err) {
    console.error('Write error:', err);
    logTerminal(`Send failed: ${err.message}`, 'error');
  }
}

async function saveConfigToBoard() {
  if (!writer) return;
  
  logTerminal('Saving settings in RAM...', 'sent');
  
  // Compile list of settings
  for (const [key, value] of Object.entries(activeConfig)) {
    // If it's pin config (check mapping)
    const isPin = ['elrs_rx', 'elrs_tx', 'esc_1', 'esc_2', 'servo_1', 'servo_2', 'cs', 'imu_sck', 'imu_mos', 'imu_miso'].includes(key);
    const paramName = isPin ? `pin_${key}` : key;
    
    // Format precision
    const valStr = isPin ? Math.round(value) : value.toFixed(6);
    
    const encoder = new TextEncoder();
    await writer.write(encoder.encode(`set ${paramName} = ${valStr}\n`));
    // Brief delay to prevent serial overrun
    await new Promise(r => setTimeout(r, 15));
  }
  
  // Commit to Flash & Reboot
  await sendCommand('save');
}

function setConnectionState(isConnected) {
  if (isConnected) {
    statusDot.className = 'status-dot connected';
    statusText.textContent = 'Connected';
    btnConnect.innerHTML = '<i data-lucide="log-out"></i> Disconnect';
    btnLoad.removeAttribute('disabled');
    btnSave.removeAttribute('disabled');
    btnDefaults.removeAttribute('disabled');
    terminalInput.removeAttribute('disabled');
  } else {
    statusDot.className = 'status-dot disconnected';
    statusText.textContent = 'Disconnected';
    btnConnect.innerHTML = '<i data-lucide="usb"></i> Connect Device';
    btnLoad.setAttribute('disabled', 'true');
    btnSave.setAttribute('disabled', 'true');
    btnDefaults.setAttribute('disabled', 'true');
    terminalInput.setAttribute('disabled', 'true');
  }
  lucide.createIcons();
}

function logTerminal(text, type = 'recv') {
  const line = document.createElement('div');
  line.className = `terminal-line ${type}`;
  line.textContent = text;
  terminalOutput.appendChild(line);
  
  // Auto Scroll
  terminalOutput.scrollTop = terminalOutput.scrollHeight;
  
  // Limit output lines to 500
  while (terminalOutput.children.length > 500) {
    terminalOutput.removeChild(terminalOutput.firstChild);
  }
}

// ------------------------------
// Three.js 3D Orientation visualizer
// ------------------------------

function init3DVisualizer() {
  const container = document.getElementById('render-canvas-container');
  const canvas = document.getElementById('imu-render-canvas');
  
  const width = container.clientWidth;
  const height = container.clientHeight;
  
  // Scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x040406);
  
  // Camera
  camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
  camera.position.set(3, 2.5, 5.5);
  camera.lookAt(0, 0, 0);
  
  // Renderer
  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  
  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
  scene.add(ambientLight);
  
  const dirLight1 = new THREE.DirectionalLight(0xffc72c, 0.8); // Maryland Gold Light
  dirLight1.position.set(5, 5, 5);
  scene.add(dirLight1);
  
  const dirLight2 = new THREE.DirectionalLight(0xc8102e, 0.6); // Maryland Red Light
  dirLight2.position.set(-5, -5, 5);
  scene.add(dirLight2);

  // Grid Helper
  const gridHelper = new THREE.GridHelper(8, 16, 0xffc72c, 0x1a1a24);
  gridHelper.position.y = -2;
  scene.add(gridHelper);

  // Static reference coordinate system (axes helper)
  const staticAxes = new THREE.AxesHelper(3.5);
  staticAxes.material.transparent = true;
  staticAxes.material.opacity = 0.4;
  scene.add(staticAxes);

  // Build a custom IMU sensor model (black rectangular prism)
  const vehicleGroup = new THREE.Group();
  
  // 1. IMU Body: black rectangular prism (2.2 x 0.4 x 1.6)
  const bodyGeo = new THREE.BoxGeometry(2.2, 0.4, 1.6);
  const bodyMat = new THREE.MeshStandardMaterial({ 
    color: 0x0c0c0d, 
    roughness: 0.4, 
    metalness: 0.8 
  });
  const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
  vehicleGroup.add(bodyMesh);
  
  // 2. Gold connector pins on sides to resemble a sensor module
  const pinGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.2, 8);
  const pinMat = new THREE.MeshStandardMaterial({
    color: 0xffc72c,
    roughness: 0.1,
    metalness: 0.9
  });
  
  // Add 4 pins along left side (-0.8 Z)
  for (let i = 0; i < 4; i++) {
    const pin = new THREE.Mesh(pinGeo, pinMat);
    pin.position.set(-0.75 + i * 0.5, -0.21, -0.8);
    vehicleGroup.add(pin);
  }
  // Add 4 pins along right side (0.8 Z)
  for (let i = 0; i < 4; i++) {
    const pin = new THREE.Mesh(pinGeo, pinMat);
    pin.position.set(-0.75 + i * 0.5, -0.21, 0.8);
    vehicleGroup.add(pin);
  }

  // 3. Canvas texture for "IMU" label
  function createTextTexture(text) {
    const canvas = document.createElement('canvas');
    canvas.width = 128;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    ctx.font = 'bold 32px "Inter", "Fira Code", sans-serif';
    ctx.fillStyle = '#ffc72c';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    
    return new THREE.CanvasTexture(canvas);
  }

  const labelGeo = new THREE.PlaneGeometry(1.2, 0.6);
  const labelMat = new THREE.MeshBasicMaterial({
    map: createTextTexture('IMU'),
    transparent: true,
    side: THREE.DoubleSide
  });
  const labelMesh = new THREE.Mesh(labelGeo, labelMat);
  labelMesh.position.set(0, 0.201, 0); // slightly above top face
  labelMesh.rotation.x = -Math.PI / 2; // flat on top
  vehicleGroup.add(labelMesh);

  // 4. Little coordinate system attached directly to the IMU (local frame)
  const localAxes = new THREE.AxesHelper(1.8);
  vehicleGroup.add(localAxes);

  vehicleMesh = vehicleGroup;
  scene.add(vehicleMesh);
  
  // Set default rotation
  update3DRotation();
  
  // Mouse tracking for interactive pan/tilt view
  let mouseX = 0;
  let mouseY = 0;
  
  container.addEventListener('mousemove', (event) => {
    const rect = container.getBoundingClientRect();
    // Normalized coordinates from -1 to 1
    mouseX = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouseY = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  });
  
  container.addEventListener('mouseleave', () => {
    mouseX = 0;
    mouseY = 0;
  });
  
  // Render Loop
  function animate() {
    requestAnimationFrame(animate);
    
    // Smoothly interpolate camera position based on mouse coordinates (pan/tilt)
    const targetX = 3 + mouseX * 2.0;
    const targetY = 2.5 + mouseY * 1.5;
    
    camera.position.x += (targetX - camera.position.x) * 0.05;
    camera.position.y += (targetY - camera.position.y) * 0.05;
    camera.lookAt(0, 0, 0);
    
    renderer.render(scene, camera);
  }
  animate();
  
  // Handle Resize
  window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  });
}

function update3DRotation() {
  if (!vehicleMesh) return;
  
  // Read euler alignment inputs (in degrees)
  const roll = activeConfig['imu_euler_x'] || 0;
  const pitch = activeConfig['imu_euler_y'] || 270; // 270 is typical default mounting
  const yaw = activeConfig['imu_euler_z'] || 0;
  
  // Convert to radians
  const rRad = THREE.MathUtils.degToRad(roll);
  const pRad = THREE.MathUtils.degToRad(pitch);
  const yRad = THREE.MathUtils.degToRad(yaw);
  
  // Set rotation order equivalent to flight controller parsing (XYZ sequential)
  vehicleMesh.rotation.set(rRad, pRad, yRad, 'XYZ');
}
