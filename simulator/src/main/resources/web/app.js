/**
 * REBUILT 2026 Simulator - Web Frontend
 * Team 3164 Stealth Tigers
 */

// ============================================================================
// CONSTANTS
// ============================================================================

const FIELD = {
    LENGTH: 16.5405,  // meters (651.2 inches)
    WIDTH: 8.0696,    // meters (317.7 inches)
    CENTER_X: 16.5405 / 2,
    CENTER_Y: 8.0696 / 2
};

// HUB positions
const HUB = {
    SIZE: 1.194,
    RED_X: FIELD.LENGTH - 4.03,
    RED_Y: FIELD.CENTER_Y,
    BLUE_X: 4.03,
    BLUE_Y: FIELD.CENTER_Y
};

// TOWER positions
const TOWER = {
    LENGTH: 1.251,
    WIDTH: 1.143,
    RED_X: FIELD.LENGTH - 2.5,
    RED_Y: FIELD.CENTER_Y + 2.5,
    BLUE_X: 2.5,
    BLUE_Y: FIELD.CENTER_Y + 2.5
};

// BUMP positions
const BUMPS = [
    { x: FIELD.CENTER_X + 3.0, y: FIELD.CENTER_Y + 2.0 },
    { x: FIELD.CENTER_X + 3.0, y: FIELD.CENTER_Y - 2.0 },
    { x: FIELD.CENTER_X - 3.0, y: FIELD.CENTER_Y + 2.0 },
    { x: FIELD.CENTER_X - 3.0, y: FIELD.CENTER_Y - 2.0 }
];

// TRENCH positions
const TRENCHES = [
    { x: FIELD.CENTER_X + 4.5, y: FIELD.CENTER_Y + 3.0 },
    { x: FIELD.CENTER_X + 4.5, y: FIELD.CENTER_Y - 3.0 },
    { x: FIELD.CENTER_X - 4.5, y: FIELD.CENTER_Y + 3.0 },
    { x: FIELD.CENTER_X - 4.5, y: FIELD.CENTER_Y - 3.0 }
];

const ROBOT = {
    LENGTH: 0.9334,  // with bumpers
    WIDTH: 0.9334
};

const FUEL = {
    RADIUS: 0.075  // 5.91 inches / 2
};

// ============================================================================
// STATE
// ============================================================================

let ws = null;
let connected = false;
let state = null;
let canvas, ctx;

// Input state
const keys = {};
let shooterAngle = 0;
let shooterPower = 0;

const input = {
    forward: 0,
    strafe: 0,
    turn: 0,
    shooterAngle: 0,
    shooterPower: 0,
    intake: false,
    shoot: false,
    spinUp: false,
    climberUp: false,
    climberDown: false,
    level1: false,
    level2: false,
    level3: false,
    toggleTrenchMode: false,
    toggleSpeed: false,
    toggleFieldRel: false,
    resetGyro: false,
    skiStop: false,
    resetRobot: false,
    redChuteRelease: false,
    blueChuteRelease: false,
    startMatch: false,
    pauseMatch: false
};

// ============================================================================
// WEBSOCKET
// ============================================================================

function connect() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        connected = true;
        updateConnectionStatus(true);
        console.log('Connected to REBUILT 2026 simulator');
    };

    ws.onclose = () => {
        connected = false;
        updateConnectionStatus(false);
        console.log('Disconnected from simulator');
        setTimeout(connect, 2000);
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'state') {
                state = data;
                updateUI();
            }
        } catch (e) {
            console.error('Error parsing message:', e);
        }
    };
}

function sendInput() {
    if (!connected || !ws) return;

    // Update continuous inputs from keys
    input.forward = (keys['KeyW'] ? 1 : 0) - (keys['KeyS'] ? 1 : 0);
    input.strafe = (keys['KeyA'] ? 1 : 0) - (keys['KeyD'] ? 1 : 0);
    input.turn = (keys['KeyQ'] ? 1 : 0) - (keys['KeyE'] ? 1 : 0);

    // Shooter angle (R/F keys)
    if (keys['KeyR']) shooterAngle = Math.min(1, shooterAngle + 0.02);
    if (keys['KeyF']) shooterAngle = Math.max(0, shooterAngle - 0.02);
    input.shooterAngle = shooterAngle;

    // Shooter power (Up/Down arrows)
    if (keys['ArrowUp']) shooterPower = Math.min(1, shooterPower + 0.02);
    if (keys['ArrowDown']) shooterPower = Math.max(0, shooterPower - 0.02);
    input.shooterPower = shooterPower;

    // Button inputs
    input.intake = keys['Space'] || false;
    input.shoot = keys['ShiftLeft'] || keys['ShiftRight'] || false;

    input.climberUp = keys['BracketRight'] || false;
    input.climberDown = keys['BracketLeft'] || false;
    input.level1 = keys['Digit1'] || false;
    input.level2 = keys['Digit2'] || false;
    input.level3 = keys['Digit3'] || false;

    input.skiStop = keys['KeyV'] || false;

    // HP controls
    input.redChuteRelease = keys['KeyZ'] || false;
    input.blueChuteRelease = keys['Slash'] || false;

    try {
        ws.send(JSON.stringify({
            type: 'input',
            ...input
        }));
    } catch (e) {
        console.error('Error sending input:', e);
    }

    // Reset one-shot inputs
    input.toggleSpeed = false;
    input.toggleFieldRel = false;
    input.toggleTrenchMode = false;
    input.resetGyro = false;
    input.resetRobot = false;
    input.startMatch = false;
    input.pauseMatch = false;
}

// ============================================================================
// INPUT HANDLING
// ============================================================================

function setupInputHandlers() {
    document.addEventListener('keydown', (e) => {
        if (e.repeat) return;

        keys[e.code] = true;

        // One-shot actions
        switch (e.code) {
            case 'KeyX':
                input.toggleSpeed = true;
                break;
            case 'KeyC':
                input.toggleFieldRel = true;
                break;
            case 'KeyT':
                input.toggleTrenchMode = true;
                break;
            case 'KeyG':
                input.resetGyro = true;
                break;
            case 'Escape':
                input.resetRobot = true;
                break;
            case 'Enter':
                input.startMatch = true;
                break;
            case 'KeyP':
                // Debug: add FUEL
                if (ws && connected) {
                    ws.send(JSON.stringify({ type: 'addFuel' }));
                }
                break;
        }

        // Prevent default for game keys
        if (['Space', 'ArrowUp', 'ArrowDown'].includes(e.code)) {
            e.preventDefault();
        }
    });

    document.addEventListener('keyup', (e) => {
        keys[e.code] = false;
    });

    window.addEventListener('blur', () => {
        Object.keys(keys).forEach(k => keys[k] = false);
    });

    // Button handlers
    document.getElementById('btn-start-match').addEventListener('click', () => {
        if (ws && connected) {
            ws.send(JSON.stringify({ type: 'startMatch' }));
        }
    });

    document.getElementById('btn-reset').addEventListener('click', () => {
        if (ws && connected) {
            ws.send(JSON.stringify({ type: 'reset' }));
        }
    });

    // Auto mode selector
    document.getElementById('auto-mode').addEventListener('change', (e) => {
        if (ws && connected) {
            const mode = parseInt(e.target.value);
            ws.send(JSON.stringify({ type: 'setAutoMode', mode: mode }));
        }
    });
}

// ============================================================================
// UI UPDATES
// ============================================================================

function updateConnectionStatus(isConnected) {
    const el = document.getElementById('connection-status');
    if (isConnected) {
        el.textContent = 'Connected';
        el.className = 'status connected';
    } else {
        el.textContent = 'Disconnected';
        el.className = 'status disconnected';
    }
}

function updateUI() {
    if (!state) return;

    // Match info
    const remaining = state.match.remaining || 0;
    const mins = Math.floor(remaining / 60);
    const secs = Math.floor(remaining % 60);
    document.getElementById('match-time').textContent = `${mins}:${secs.toString().padStart(2, '0')}`;
    document.getElementById('match-phase').textContent = state.match.phaseName || 'Pre-Match';

    // Auto mode info
    const autoSelect = document.getElementById('auto-mode');
    const autoStatus = document.getElementById('auto-status');

    // Update dropdown value if not locked (and not currently focused)
    if (!state.match.autoLocked && document.activeElement !== autoSelect) {
        autoSelect.value = state.match.autoMode;
    }

    // Disable dropdown when locked (match started)
    autoSelect.disabled = state.match.autoLocked;

    // Show auto phase during AUTO
    if (state.match.phase === 'AUTO') {
        autoStatus.textContent = state.match.autoPhase || '';
        autoStatus.className = 'auto-status running';
    } else if (state.match.autoLocked) {
        autoStatus.textContent = 'Locked';
        autoStatus.className = 'auto-status locked';
    } else {
        autoStatus.textContent = '';
        autoStatus.className = 'auto-status';
    }

    // Scores
    document.getElementById('red-score').textContent = state.match.scores.redTotal;
    document.getElementById('blue-score').textContent = state.match.scores.blueTotal;

    // HUB status
    const redHub = document.getElementById('red-hub-status');
    const blueHub = document.getElementById('blue-hub-status');
    redHub.className = `hub-status ${state.match.redHubActive ? 'active' : 'inactive'}`;
    blueHub.className = `hub-status ${state.match.blueHubActive ? 'active' : 'inactive'}`;

    // Shooter
    const anglePercent = (state.shooter.angle / 75) * 100;
    const velocityPercent = (state.shooter.velocity / 20) * 100;
    document.getElementById('shooter-angle').textContent = `${state.shooter.angle.toFixed(1)}°`;
    document.getElementById('shooter-angle-bar').style.width = `${anglePercent}%`;
    document.getElementById('shooter-velocity').textContent = `${state.shooter.velocity.toFixed(1)} m/s`;
    document.getElementById('shooter-velocity-bar').style.width = `${velocityPercent}%`;
    document.getElementById('fuel-count').textContent = `${state.shooter.fuelCount} / 8`;
    document.getElementById('shooter-status').textContent = state.shooter.intakeState;

    // FUEL indicator
    const fuelIndicator = document.getElementById('fuel-indicator');
    fuelIndicator.innerHTML = '';
    for (let i = 0; i < 8; i++) {
        const dot = document.createElement('div');
        dot.className = `fuel-dot ${i < state.shooter.fuelCount ? 'filled' : ''}`;
        fuelIndicator.appendChild(dot);
    }

    // Climber
    const climberPercent = (state.climber.position / 1.7) * 100;
    document.getElementById('climber-position').textContent = `${state.climber.position.toFixed(2)}m`;
    document.getElementById('climber-bar').style.width = `${climberPercent}%`;

    // Climb level indicators
    for (let i = 1; i <= 3; i++) {
        const el = document.getElementById(`climb-l${i}`);
        const isTarget = state.climber.level === i;
        const isComplete = state.climber.complete && state.climber.level === i;
        el.className = `climb-level ${isTarget ? 'target' : ''} ${isComplete ? 'complete' : ''}`;
    }

    // Robot info
    document.getElementById('robot-position').textContent =
        `(${state.robot.x.toFixed(1)}, ${state.robot.y.toFixed(1)})`;
    document.getElementById('robot-heading').textContent = `${state.robot.heading.toFixed(0)}°`;

    const speed = Math.hypot(state.robot.vx, state.robot.vy);
    document.getElementById('robot-speed').textContent = `${speed.toFixed(1)} m/s`;
    document.getElementById('current-command').textContent = state.control.command;

    // Mode indicators
    document.getElementById('mode-field-rel').className =
        `mode-indicator ${state.control.fieldRelative ? 'active' : ''}`;
    document.getElementById('mode-slow').className =
        `mode-indicator ${state.control.slowMode ? 'active' : ''}`;
    document.getElementById('mode-trench').className =
        `mode-indicator ${state.robot.trenchMode ? 'active' : ''}`;

    // Render field
    renderField();
}

// ============================================================================
// FIELD RENDERING
// ============================================================================

function initCanvas() {
    canvas = document.getElementById('field-canvas');
    ctx = canvas.getContext('2d');

    function resize() {
        const container = canvas.parentElement;
        const width = container.clientWidth - 20;
        const height = width * (FIELD.WIDTH / FIELD.LENGTH);

        canvas.width = width;
        canvas.height = height;

        if (state) renderField();
    }

    window.addEventListener('resize', resize);
    resize();
}

function renderField() {
    if (!ctx || !state) return;

    const w = canvas.width;
    const h = canvas.height;
    const scale = w / FIELD.LENGTH;

    // Clear
    ctx.fillStyle = '#1a2634';
    ctx.fillRect(0, 0, w, h);

    // Field border
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.strokeRect(2, 2, w - 4, h - 4);

    // Center line
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.setLineDash([10, 5]);
    ctx.beginPath();
    ctx.moveTo(w / 2, 0);
    ctx.lineTo(w / 2, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw HUBs
    drawHub(HUB.RED_X * scale, h - HUB.RED_Y * scale, HUB.SIZE * scale, 'red', state.match.redHubActive);
    drawHub(HUB.BLUE_X * scale, h - HUB.BLUE_Y * scale, HUB.SIZE * scale, 'blue', state.match.blueHubActive);

    // Draw TOWERs
    drawTower(TOWER.RED_X * scale, h - TOWER.RED_Y * scale, TOWER.LENGTH * scale, TOWER.WIDTH * scale, 'red');
    drawTower(TOWER.BLUE_X * scale, h - TOWER.BLUE_Y * scale, TOWER.LENGTH * scale, TOWER.WIDTH * scale, 'blue');

    // Draw BUMPs
    BUMPS.forEach(bump => {
        drawBump(bump.x * scale, h - bump.y * scale, 1.854 * scale, 1.128 * scale);
    });

    // Draw TRENCHes
    TRENCHES.forEach(trench => {
        drawTrench(trench.x * scale, h - trench.y * scale, 1.668 * scale, 1.194 * scale);
    });

    // Draw FUEL on field
    if (state.fuel && state.fuel.field) {
        state.fuel.field.forEach(fuel => {
            drawFuel(fuel.x * scale, h - fuel.y * scale, FUEL.RADIUS * scale, fuel.moving);
        });
    }

    // Draw FUEL in flight
    if (state.fuel && state.fuel.flight) {
        state.fuel.flight.forEach(fuel => {
            drawFuel(fuel.x * scale, h - fuel.y * scale, FUEL.RADIUS * scale * (1 + fuel.z * 0.2), true);
        });
    }

    // Draw robot
    drawRobot(
        state.robot.x * scale,
        h - state.robot.y * scale,
        -state.robot.heading * Math.PI / 180,
        scale
    );

    // Draw swerve modules
    drawSwerveModules(
        state.robot.x * scale,
        h - state.robot.y * scale,
        -state.robot.heading * Math.PI / 180,
        scale
    );
}

function drawHub(x, y, size, alliance, isActive) {
    const color = alliance === 'red' ? '#e94560' : '#3498db';
    const halfSize = size / 2;

    // HUB body
    ctx.fillStyle = isActive ? color : 'rgba(100, 100, 100, 0.5)';
    ctx.strokeStyle = isActive ? '#fff' : '#666';
    ctx.lineWidth = isActive ? 3 : 1;

    ctx.fillRect(x - halfSize, y - halfSize, size, size);
    ctx.strokeRect(x - halfSize, y - halfSize, size, size);

    // HUB label
    ctx.fillStyle = isActive ? '#fff' : '#888';
    ctx.font = 'bold 14px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('HUB', x, y);

    // Glow effect when active
    if (isActive) {
        ctx.shadowColor = color;
        ctx.shadowBlur = 15;
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.strokeRect(x - halfSize - 2, y - halfSize - 2, size + 4, size + 4);
        ctx.shadowBlur = 0;
    }
}

function drawTower(x, y, length, width, alliance) {
    const color = alliance === 'red' ? '#c0392b' : '#2980b9';
    const halfL = length / 2;
    const halfW = width / 2;

    ctx.fillStyle = color;
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;

    ctx.fillRect(x - halfL, y - halfW, length, width);
    ctx.strokeRect(x - halfL, y - halfW, length, width);

    // TOWER label
    ctx.fillStyle = '#fff';
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('TOWER', x, y);
}

function drawBump(x, y, length, width) {
    const halfL = length / 2;
    const halfW = width / 2;

    ctx.fillStyle = 'rgba(255, 200, 100, 0.4)';
    ctx.strokeStyle = '#ffc864';
    ctx.lineWidth = 2;

    ctx.fillRect(x - halfL, y - halfW, length, width);
    ctx.strokeRect(x - halfL, y - halfW, length, width);
}

function drawTrench(x, y, length, width) {
    const halfL = length / 2;
    const halfW = width / 2;

    ctx.fillStyle = 'rgba(100, 100, 100, 0.4)';
    ctx.strokeStyle = '#888';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 3]);

    ctx.fillRect(x - halfL, y - halfW, length, width);
    ctx.strokeRect(x - halfL, y - halfW, length, width);

    ctx.setLineDash([]);
}

function drawFuel(x, y, radius, isMoving) {
    ctx.fillStyle = isMoving ? '#ff9f1c' : '#f77f00';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
}

function drawRobot(x, y, heading, scale) {
    const robotW = ROBOT.WIDTH * scale;
    const robotH = ROBOT.LENGTH * scale;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading);

    // Robot body
    const alliance = state.robot.alliance;
    ctx.fillStyle = alliance === 'RED' ? '#e94560' : '#3498db';
    ctx.fillRect(-robotW / 2, -robotH / 2, robotW, robotH);

    // Robot outline
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.strokeRect(-robotW / 2, -robotH / 2, robotW, robotH);

    // Direction arrow
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.moveTo(0, -robotH / 2 - 5);
    ctx.lineTo(-8, -robotH / 2 + 10);
    ctx.lineTo(8, -robotH / 2 + 10);
    ctx.closePath();
    ctx.fill();

    // Team number
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 12px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('3164', 0, 0);

    // FUEL count indicator
    if (state.shooter.fuelCount > 0) {
        ctx.fillStyle = '#f77f00';
        ctx.font = 'bold 10px sans-serif';
        ctx.fillText(`${state.shooter.fuelCount}`, 0, 12);
    }

    ctx.restore();
}

function drawSwerveModules(x, y, heading, scale) {
    if (!state.swerve) return;

    const halfW = ROBOT.WIDTH * scale / 2 - 5;
    const halfH = ROBOT.LENGTH * scale / 2 - 5;

    const modulePositions = [
        { x: halfH, y: halfW },
        { x: halfH, y: -halfW },
        { x: -halfH, y: halfW },
        { x: -halfH, y: -halfW }
    ];

    const modules = ['fl', 'fr', 'rl', 'rr'];

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading);

    modules.forEach((name, i) => {
        const mod = state.swerve[name];
        const pos = modulePositions[i];

        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-mod.angle * Math.PI / 180);

        const wheelLength = 15;
        const wheelWidth = 6;

        ctx.fillStyle = '#333';
        ctx.fillRect(-wheelLength / 2, -wheelWidth / 2, wheelLength, wheelWidth);

        const speedScale = Math.min(Math.abs(mod.speed) / 4, 1);
        ctx.strokeStyle = mod.speed >= 0 ? '#0f0' : '#f00';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(speedScale * 15 * Math.sign(mod.speed || 1), 0);
        ctx.stroke();

        ctx.restore();
    });

    ctx.restore();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

function init() {
    initCanvas();
    setupInputHandlers();
    connect();

    setInterval(sendInput, 20);

    console.log('REBUILT 2026 Simulator initialized');
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
