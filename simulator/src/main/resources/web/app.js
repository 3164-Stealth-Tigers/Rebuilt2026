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
    CENTER_Y: 8.0696 / 2,
    // Alliance Zone boundary (158.6 inches from wall)
    ALLIANCE_ZONE_DEPTH: 4.03
};

// HUB positions (centered in Alliance Zone, 4.03m from wall)
const HUB = {
    SIZE: 1.194,
    RED_X: FIELD.LENGTH - 4.03,
    RED_Y: FIELD.CENTER_Y,
    BLUE_X: 4.03,
    BLUE_Y: FIELD.CENTER_Y
};

// TOWER positions (at alliance walls, offset in Y)
const TOWER = {
    LENGTH: 1.251,
    WIDTH: 1.143,
    RED_X: FIELD.LENGTH - 0.625,  // Against wall
    RED_Y: FIELD.CENTER_Y + 2.0,
    BLUE_X: 0.625,                 // Against wall
    BLUE_Y: FIELD.CENTER_Y + 2.0
};

// BUMP positions (flanking HUBs on either side in Y)
// BUMPs are 1.854m (Y) x 1.128m (X) - form barrier with HUB and TRENCH
// Game manual: "BUMPS are structures on either side of the HUB"
const BUMP = {
    LENGTH: 1.854,  // Y direction (along barrier line)
    WIDTH: 1.128    // X direction (depth into field)
};
const BUMPS = [
    // Red side bumps (flanking red HUB, at same X as HUB)
    { x: HUB.RED_X, y: HUB.RED_Y + HUB.SIZE/2 + BUMP.LENGTH/2, alliance: 'red' },   // Above HUB
    { x: HUB.RED_X, y: HUB.RED_Y - HUB.SIZE/2 - BUMP.LENGTH/2, alliance: 'red' },   // Below HUB
    // Blue side bumps (flanking blue HUB, at same X as HUB)
    { x: HUB.BLUE_X, y: HUB.BLUE_Y + HUB.SIZE/2 + BUMP.LENGTH/2, alliance: 'blue' }, // Above HUB
    { x: HUB.BLUE_X, y: HUB.BLUE_Y - HUB.SIZE/2 - BUMP.LENGTH/2, alliance: 'blue' }  // Below HUB
];

// TRENCH positions (extending from guardrail to BUMP)
// TRENCHes are 1.668m (Y) x 1.194m (X)
// Game manual: "The TRENCH extends from the guardrail to the BUMP on both sides of the FIELD"
const TRENCH = {
    LENGTH: 1.668,  // Y direction (along barrier line)
    WIDTH: 1.194    // X direction (depth into field)
};
const TRENCHES = [
    // Red side trenches (at same X as HUB, extending to guardrails)
    { x: HUB.RED_X, y: FIELD.WIDTH - TRENCH.LENGTH/2, alliance: 'red' },  // Top guardrail
    { x: HUB.RED_X, y: TRENCH.LENGTH/2, alliance: 'red' },                 // Bottom guardrail
    // Blue side trenches (at same X as HUB, extending to guardrails)
    { x: HUB.BLUE_X, y: FIELD.WIDTH - TRENCH.LENGTH/2, alliance: 'blue' }, // Top guardrail
    { x: HUB.BLUE_X, y: TRENCH.LENGTH/2, alliance: 'blue' }                // Bottom guardrail
];

// DEPOT positions (near alliance walls)
const DEPOT = {
    LENGTH: 1.07,
    WIDTH: 0.686,
    RED_X: FIELD.LENGTH - 1.0,
    RED_Y: 1.5,
    BLUE_X: 1.0,
    BLUE_Y: 1.5
};

const ROBOT = {
    LENGTH: 0.9334,  // with bumpers
    WIDTH: 0.9334
};

const FUEL = {
    RADIUS: 0.075  // 5.91 inches / 2
};

// AprilTag positions: {id, x, y, z, rotation}
// Rotation: 0 = facing +X (red wall), 90 = facing +Y, 180 = facing -X (blue wall), 270 = facing -Y
const APRILTAGS = [
    // BLUE HUB AprilTags (Tags 1-3) - facing toward field center (+X direction)
    {id: 1, x: HUB.BLUE_X + HUB.SIZE/2 + 0.01, y: FIELD.CENTER_Y, z: 1.2, rotation: 0},
    {id: 2, x: HUB.BLUE_X + HUB.SIZE/2 + 0.01, y: FIELD.CENTER_Y + 0.4, z: 1.0, rotation: 0},
    {id: 3, x: HUB.BLUE_X + HUB.SIZE/2 + 0.01, y: FIELD.CENTER_Y - 0.4, z: 1.0, rotation: 0},

    // RED HUB AprilTags (Tags 4-6) - facing toward field center (-X direction)
    {id: 4, x: HUB.RED_X - HUB.SIZE/2 - 0.01, y: FIELD.CENTER_Y, z: 1.2, rotation: 180},
    {id: 5, x: HUB.RED_X - HUB.SIZE/2 - 0.01, y: FIELD.CENTER_Y + 0.4, z: 1.0, rotation: 180},
    {id: 6, x: HUB.RED_X - HUB.SIZE/2 - 0.01, y: FIELD.CENTER_Y - 0.4, z: 1.0, rotation: 180},

    // BLUE TOWER AprilTags (Tags 7-8)
    {id: 7, x: TOWER.BLUE_X + 0.3, y: TOWER.BLUE_Y, z: 1.5, rotation: 0},
    {id: 8, x: TOWER.BLUE_X, y: TOWER.BLUE_Y - 0.5, z: 1.5, rotation: 270},

    // RED TOWER AprilTags (Tags 9-10)
    {id: 9, x: TOWER.RED_X - 0.3, y: TOWER.RED_Y, z: 1.5, rotation: 180},
    {id: 10, x: TOWER.RED_X, y: TOWER.RED_Y - 0.5, z: 1.5, rotation: 270},

    // FIELD WALL AprilTags (Tags 11-16) - for general field localization
    {id: 11, x: 0.01, y: FIELD.WIDTH/4, z: 0.6, rotation: 0},
    {id: 12, x: 0.01, y: 3*FIELD.WIDTH/4, z: 0.6, rotation: 0},
    {id: 13, x: FIELD.LENGTH - 0.01, y: FIELD.WIDTH/4, z: 0.6, rotation: 180},
    {id: 14, x: FIELD.LENGTH - 0.01, y: 3*FIELD.WIDTH/4, z: 0.6, rotation: 180},
    {id: 15, x: FIELD.CENTER_X, y: 0.01, z: 0.6, rotation: 90},
    {id: 16, x: FIELD.CENTER_X, y: FIELD.WIDTH - 0.01, z: 0.6, rotation: 270}
];

// ============================================================================
// STATE
// ============================================================================

let ws = null;
let connected = false;
let state = null;
let canvas, ctx;
let fpvCanvas, fpvCtx;
let fpvCanvasLeft, fpvCtxLeft;
let fpvCanvasRight, fpvCtxRight;

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
            // Clear robot lineup so it gets recreated with fresh data
            document.getElementById('blue-lineup').innerHTML = '';
            document.getElementById('red-lineup').innerHTML = '';
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

    // Score breakdown
    document.getElementById('red-fuel').textContent = `FUEL: ${state.match.scores.redFuel}`;
    document.getElementById('blue-fuel').textContent = `FUEL: ${state.match.scores.blueFuel}`;
    document.getElementById('red-tower').textContent = `TOWER: ${state.match.scores.redTower}`;
    document.getElementById('blue-tower').textContent = `TOWER: ${state.match.scores.blueTower}`;

    // Ranking Points indicators
    const redEnergized = document.getElementById('red-energized');
    const blueEnergized = document.getElementById('blue-energized');
    const redTraversal = document.getElementById('red-traversal');
    const blueTraversal = document.getElementById('blue-traversal');

    if (state.match.rankingPoints) {
        redEnergized.className = `rp-badge ${state.match.rankingPoints.redEnergized ? 'earned' : ''}`;
        blueEnergized.className = `rp-badge ${state.match.rankingPoints.blueEnergized ? 'earned' : ''}`;
        redTraversal.className = `rp-badge ${state.match.rankingPoints.redTraversal ? 'earned' : ''}`;
        blueTraversal.className = `rp-badge ${state.match.rankingPoints.blueTraversal ? 'earned' : ''}`;
    }

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

    // Update robot lineup
    updateRobotLineup();

    // Render field
    renderField();

    // Render first-person view (all cameras)
    renderAllFPV();
}

/**
 * Update the robot lineup panels with auto mode info.
 * Only recreates cards if the lineup hasn't been initialized.
 */
function updateRobotLineup() {
    if (!state || !state.multiRobotEnabled || !state.allRobots) return;

    const blueLineup = document.getElementById('blue-lineup');
    const redLineup = document.getElementById('red-lineup');

    // Check if we need to create the cards (first time or reset)
    const needsInit = blueLineup.children.length === 0;

    if (needsInit) {
        // Clear and recreate
        blueLineup.innerHTML = '';
        redLineup.innerHTML = '';

        // Sort robots by alliance
        const blueRobots = state.allRobots.filter(r => r.alliance === 'BLUE');
        const redRobots = state.allRobots.filter(r => r.alliance === 'RED');

        // Create lineup cards for blue alliance
        blueRobots.forEach(robot => {
            blueLineup.appendChild(createRobotCard(robot, 'blue'));
        });

        // Create lineup cards for red alliance
        redRobots.forEach(robot => {
            redLineup.appendChild(createRobotCard(robot, 'red'));
        });
    } else {
        // Just update the existing dropdowns with current values
        state.allRobots.forEach(robot => {
            if (!robot.isPlayer) {
                const select = document.querySelector(`.ai-auto-select[data-robot-id="${robot.id}"]`);
                if (select && document.activeElement !== select) {
                    // Only update if user isn't currently interacting with it
                    const serverValue = robot.autoModeIndex;
                    if (serverValue >= 0 && select.value !== String(serverValue)) {
                        select.value = serverValue;
                    }
                }
                // Update disabled state based on match lock
                if (select) {
                    select.disabled = state.match.autoLocked;
                }
            }
        });
    }
}

/**
 * Create a robot card for the lineup.
 */
function createRobotCard(robot, alliance) {
    const card = document.createElement('div');
    card.className = `lineup-robot ${alliance}${robot.isPlayer ? ' player' : ''}`;

    let html = `<span class="team-number">${robot.teamNumber}</span>`;

    if (robot.isPlayer) {
        html += `<span class="player-badge">YOU</span>`;
    } else {
        // Add auto mode dropdown for AI robots (20 modes)
        const autoModes = [
            // Original Modes (0-9)
            { value: 0, name: 'Do Nothing' },
            { value: 1, name: 'Score & Collect' },
            { value: 2, name: 'Quick Climb' },
            { value: 3, name: 'Score Then Climb' },
            { value: 4, name: 'Depot Raid' },
            { value: 5, name: 'Far Neutral' },
            { value: 6, name: 'Preload Only' },
            { value: 7, name: 'Max Cycles' },
            { value: 8, name: 'Climb Support' },
            { value: 9, name: 'Win AUTO' },
            // Optimized Modes (10-14)
            { value: 10, name: 'Score+Collect+Climb' },
            { value: 11, name: 'Fast Climb' },
            { value: 12, name: 'Balanced' },
            { value: 13, name: 'Depot+Climb OPTIMAL' },
            { value: 14, name: 'Max Points' },
            // Strategic Modes (15-19)
            { value: 15, name: 'Safe Climb' },
            { value: 16, name: 'Dual Cycle' },
            { value: 17, name: 'Deny FUEL' },
            { value: 18, name: 'Center Control' },
            { value: 19, name: 'Alliance Support' }
        ];

        // Use the numeric autoModeIndex from server
        const currentMode = robot.autoModeIndex >= 0 ? robot.autoModeIndex : 0;

        html += `<select class="ai-auto-select" data-robot-id="${robot.id}" ${state.match.autoLocked ? 'disabled' : ''}>`;
        autoModes.forEach(mode => {
            const selected = mode.value === currentMode ? 'selected' : '';
            html += `<option value="${mode.value}" ${selected}>${mode.name}</option>`;
        });
        html += `</select>`;

        // Show teleop behavior
        const behavior = robot.teleopBehavior || '';
        if (behavior && behavior !== 'PLAYER') {
            html += `<span class="teleop-behavior">${behavior}</span>`;
        }
    }

    card.innerHTML = html;

    // Add event listener for auto mode change
    if (!robot.isPlayer) {
        const select = card.querySelector('.ai-auto-select');
        if (select) {
            select.addEventListener('change', (e) => {
                const robotId = parseInt(e.target.dataset.robotId);
                const mode = parseInt(e.target.value);
                setAIAutoMode(robotId, mode);
            });
        }
    }

    return card;
}

/**
 * Set the autonomous mode for an AI robot.
 */
function setAIAutoMode(robotId, mode) {
    if (ws && connected) {
        ws.send(JSON.stringify({
            type: 'setAIAutoMode',
            robotId: robotId,
            mode: mode
        }));
    }
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

    // Clear with neutral zone color
    ctx.fillStyle = '#2d3a4a';  // Neutral zone (darker)
    ctx.fillRect(0, 0, w, h);

    // Draw Alliance Zones (colored backgrounds)
    const allianceZoneWidth = FIELD.ALLIANCE_ZONE_DEPTH * scale;

    // Blue Alliance Zone (left side)
    ctx.fillStyle = 'rgba(52, 152, 219, 0.15)';  // Light blue tint
    ctx.fillRect(0, 0, allianceZoneWidth, h);

    // Red Alliance Zone (right side)
    ctx.fillStyle = 'rgba(231, 76, 60, 0.15)';  // Light red tint
    ctx.fillRect(w - allianceZoneWidth, 0, allianceZoneWidth, h);

    // Neutral Zone (center) - slightly different shade
    ctx.fillStyle = 'rgba(241, 196, 15, 0.08)';  // Very light yellow/tan
    ctx.fillRect(allianceZoneWidth, 0, w - 2 * allianceZoneWidth, h);

    // Alliance Zone boundary lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 4]);
    // Blue zone line
    ctx.beginPath();
    ctx.moveTo(allianceZoneWidth, 0);
    ctx.lineTo(allianceZoneWidth, h);
    ctx.stroke();
    // Red zone line
    ctx.beginPath();
    ctx.moveTo(w - allianceZoneWidth, 0);
    ctx.lineTo(w - allianceZoneWidth, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // Field border
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 3;
    ctx.strokeRect(2, 2, w - 4, h - 4);

    // Center line
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)';
    ctx.lineWidth = 2;
    ctx.setLineDash([10, 5]);
    ctx.beginPath();
    ctx.moveTo(w / 2, 0);
    ctx.lineTo(w / 2, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw TRENCHes (under other elements)
    // TRENCH.WIDTH = X extent (horizontal), TRENCH.LENGTH = Y extent (vertical)
    TRENCHES.forEach(trench => {
        drawTrench(trench.x * scale, h - trench.y * scale, TRENCH.WIDTH * scale, TRENCH.LENGTH * scale, trench.alliance);
    });

    // Draw BUMPs
    // BUMP.WIDTH = X extent (horizontal), BUMP.LENGTH = Y extent (vertical)
    BUMPS.forEach(bump => {
        drawBump(bump.x * scale, h - bump.y * scale, BUMP.WIDTH * scale, BUMP.LENGTH * scale, bump.alliance);
    });

    // Draw TOWERs
    drawTower(TOWER.RED_X * scale, h - TOWER.RED_Y * scale, TOWER.LENGTH * scale, TOWER.WIDTH * scale, 'red');
    drawTower(TOWER.BLUE_X * scale, h - TOWER.BLUE_Y * scale, TOWER.LENGTH * scale, TOWER.WIDTH * scale, 'blue');

    // Draw HUBs
    drawHub(HUB.RED_X * scale, h - HUB.RED_Y * scale, HUB.SIZE * scale, 'red', state.match.redHubActive);
    drawHub(HUB.BLUE_X * scale, h - HUB.BLUE_Y * scale, HUB.SIZE * scale, 'blue', state.match.blueHubActive);

    // Draw AprilTags
    APRILTAGS.forEach(tag => {
        drawAprilTag(tag.x * scale, h - tag.y * scale, 8, tag.id);
    });

    // Draw DEPOTs
    drawDepot(DEPOT.RED_X * scale, h - DEPOT.RED_Y * scale, DEPOT.LENGTH * scale, DEPOT.WIDTH * scale, 'red');
    drawDepot(DEPOT.BLUE_X * scale, h - DEPOT.BLUE_Y * scale, DEPOT.LENGTH * scale, DEPOT.WIDTH * scale, 'blue');

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

    // Draw all robots
    if (state.multiRobotEnabled && state.allRobots) {
        // Draw all robots in multi-robot mode
        state.allRobots.forEach(robot => {
            drawMultiRobot(
                robot.x * scale,
                h - robot.y * scale,
                -robot.heading * Math.PI / 180,
                scale,
                robot
            );
        });
    } else {
        // Single robot mode - draw player's robot
        drawRobot(
            state.robot.x * scale,
            h - state.robot.y * scale,
            -state.robot.heading * Math.PI / 180,
            scale
        );

        // Draw swerve modules for player
        drawSwerveModules(
            state.robot.x * scale,
            h - state.robot.y * scale,
            -state.robot.heading * Math.PI / 180,
            scale
        );
    }
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

function drawAprilTag(x, y, size, id) {
    const half = size / 2;

    // Outer black square
    ctx.fillStyle = '#000';
    ctx.fillRect(x - half, y - half, size, size);

    // Inner white border
    ctx.fillStyle = '#fff';
    ctx.fillRect(x - half + 1, y - half + 1, size - 2, size - 2);

    // Inner black square
    ctx.fillStyle = '#000';
    ctx.fillRect(x - half + 2, y - half + 2, size - 4, size - 4);

    // Tag ID number
    ctx.fillStyle = '#0f0';
    ctx.font = 'bold 7px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(id.toString(), x, y);
}

function drawBump(x, y, width, height, alliance) {
    // width = X extent (horizontal), height = Y extent (vertical)
    const halfW = width / 2;
    const halfH = height / 2;

    // Color based on alliance zone
    const fillColor = alliance === 'red' ? 'rgba(255, 180, 100, 0.5)' : 'rgba(100, 180, 255, 0.5)';
    const strokeColor = alliance === 'red' ? '#ffa500' : '#5dade2';

    ctx.fillStyle = fillColor;
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 2;

    ctx.fillRect(x - halfW, y - halfH, width, height);
    ctx.strokeRect(x - halfW, y - halfH, width, height);

    // Label
    ctx.fillStyle = '#fff';
    ctx.font = '9px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('BUMP', x, y);
}

function drawTrench(x, y, width, height, alliance) {
    // width = X extent (horizontal), height = Y extent (vertical)
    const halfW = width / 2;
    const halfH = height / 2;

    // Darker color to show it's a structure to drive under
    const fillColor = alliance === 'red' ? 'rgba(139, 69, 19, 0.6)' : 'rgba(70, 130, 180, 0.6)';
    const strokeColor = alliance === 'red' ? '#8b4513' : '#4682b4';

    ctx.fillStyle = fillColor;
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 3]);

    ctx.fillRect(x - halfW, y - halfH, width, height);
    ctx.strokeRect(x - halfW, y - halfH, width, height);

    ctx.setLineDash([]);

    // Label
    ctx.fillStyle = '#fff';
    ctx.font = '8px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('TRENCH', x, y);
}

function drawDepot(x, y, length, width, alliance) {
    const halfL = length / 2;
    const halfW = width / 2;

    const fillColor = alliance === 'red' ? 'rgba(231, 76, 60, 0.4)' : 'rgba(52, 152, 219, 0.4)';
    const strokeColor = alliance === 'red' ? '#e74c3c' : '#3498db';

    ctx.fillStyle = fillColor;
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 2;

    ctx.fillRect(x - halfL, y - halfW, length, width);
    ctx.strokeRect(x - halfL, y - halfW, length, width);

    // Label
    ctx.fillStyle = '#fff';
    ctx.font = '8px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('DEPOT', x, y);
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

/**
 * Draw a robot in multi-robot mode with team number and status.
 */
function drawMultiRobot(x, y, heading, scale, robotData) {
    const robotW = ROBOT.WIDTH * scale;
    const robotH = ROBOT.LENGTH * scale;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading);

    // Robot body color based on alliance
    let fillColor;
    let strokeColor;

    if (robotData.alliance === 'RED') {
        fillColor = robotData.isPlayer ? '#ff6b6b' : '#c0392b';
        strokeColor = robotData.isPlayer ? '#fff' : '#e74c3c';
    } else {
        fillColor = robotData.isPlayer ? '#5dade2' : '#2980b9';
        strokeColor = robotData.isPlayer ? '#fff' : '#3498db';
    }

    // Draw robot body
    ctx.fillStyle = fillColor;
    ctx.fillRect(-robotW / 2, -robotH / 2, robotW, robotH);

    // Robot outline (thicker for player)
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = robotData.isPlayer ? 3 : 2;
    ctx.strokeRect(-robotW / 2, -robotH / 2, robotW, robotH);

    // Player indicator glow
    if (robotData.isPlayer) {
        ctx.shadowColor = '#fff';
        ctx.shadowBlur = 10;
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.strokeRect(-robotW / 2 - 2, -robotH / 2 - 2, robotW + 4, robotH + 4);
        ctx.shadowBlur = 0;
    }

    // Direction arrow
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.moveTo(0, -robotH / 2 - 4);
    ctx.lineTo(-6, -robotH / 2 + 8);
    ctx.lineTo(6, -robotH / 2 + 8);
    ctx.closePath();
    ctx.fill();

    // Team number
    ctx.fillStyle = '#fff';
    ctx.font = robotData.isPlayer ? 'bold 11px sans-serif' : '10px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(robotData.teamNumber.toString(), 0, -2);

    // FUEL count indicator
    if (robotData.fuelCount > 0) {
        ctx.fillStyle = '#f77f00';
        ctx.font = 'bold 9px sans-serif';
        ctx.fillText(`${robotData.fuelCount}`, 0, 10);
    }

    // Climbing indicator
    if (robotData.isClimbing || robotData.climbComplete) {
        ctx.fillStyle = robotData.climbComplete ? '#2ecc71' : '#f39c12';
        ctx.font = 'bold 8px sans-serif';
        ctx.fillText(`L${robotData.climbLevel}`, 0, robotH / 2 + 10);
    }

    ctx.restore();

    // Draw status label below robot (only for AI robots)
    if (!robotData.isPlayer && robotData.command && robotData.command.length > 0) {
        ctx.save();
        ctx.fillStyle = robotData.alliance === 'RED' ? '#ffb3b3' : '#b3d9ff';
        ctx.font = '9px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(robotData.command, x, y + robotH / 2 + 18);
        ctx.restore();
    }
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
// FIRST PERSON VIEW RENDERING
// ============================================================================

/**
 * Initialize the FPV canvases (front, left, right cameras).
 */
function initFPVCanvas() {
    // Front camera (180° FOV)
    fpvCanvas = document.getElementById('fpv-canvas');
    if (fpvCanvas) {
        fpvCtx = fpvCanvas.getContext('2d');
    }

    // Left camera (90° FOV)
    fpvCanvasLeft = document.getElementById('fpv-canvas-left');
    if (fpvCanvasLeft) {
        fpvCtxLeft = fpvCanvasLeft.getContext('2d');
    }

    // Right camera (90° FOV)
    fpvCanvasRight = document.getElementById('fpv-canvas-right');
    if (fpvCanvasRight) {
        fpvCtxRight = fpvCanvasRight.getContext('2d');
    }

    function resizeFPV() {
        // Front camera sizing
        if (fpvCanvas) {
            fpvCanvas.width = 500;
            fpvCanvas.height = 250;
        }
        // Side cameras sizing
        if (fpvCanvasLeft) {
            fpvCanvasLeft.width = 250;
            fpvCanvasLeft.height = 200;
        }
        if (fpvCanvasRight) {
            fpvCanvasRight.width = 250;
            fpvCanvasRight.height = 200;
        }

        if (state) renderAllFPV();
    }

    window.addEventListener('resize', resizeFPV);
    resizeFPV();
}

/**
 * Render all three FPV cameras.
 */
function renderAllFPV() {
    if (!state) return;

    // Get player robot data
    let playerRobot = null;
    if (state.multiRobotEnabled && state.allRobots) {
        playerRobot = state.allRobots.find(r => r.isPlayer);
    }
    if (!playerRobot) {
        playerRobot = {
            x: state.robot.x,
            y: state.robot.y,
            heading: state.robot.heading,
            alliance: state.robot.alliance,
            fuelCount: state.shooter.fuelCount
        };
    }

    // Collect all objects once for all cameras
    const objects = collectFPVObjects();

    const robotX = playerRobot.x;
    const robotY = playerRobot.y;
    const robotHeading = -playerRobot.heading * Math.PI / 180;

    // Render front camera (180° FOV) and get visible objects
    let frontVisible = [];
    if (fpvCtx && fpvCanvas) {
        frontVisible = renderFPVCamera(fpvCanvas, fpvCtx, objects, playerRobot, Math.PI, 0);
    }

    // Render left camera (90° FOV, looking 90° left)
    let leftVisible = [];
    if (fpvCtxLeft && fpvCanvasLeft) {
        leftVisible = renderFPVCamera(fpvCanvasLeft, fpvCtxLeft, objects, playerRobot, Math.PI / 2, Math.PI / 2);
    }

    // Render right camera (90° FOV, looking 90° right)
    let rightVisible = [];
    if (fpvCtxRight && fpvCanvasRight) {
        rightVisible = renderFPVCamera(fpvCanvasRight, fpvCtxRight, objects, playerRobot, Math.PI / 2, -Math.PI / 2);
    }

    // Update AprilTag indicators for each camera
    updateAprilTagIndicator('fpv-tags-front', frontVisible);
    updateAprilTagIndicator('fpv-tags-left', leftVisible);
    updateAprilTagIndicator('fpv-tags-right', rightVisible);

    // Update HUD elements
    updateFPVHUD(playerRobot, frontVisible);
}

/**
 * Collect all objects to render in FPV views.
 */
function collectFPVObjects() {
    const objects = [];

    // Add HUBs
    objects.push({
        type: 'hub',
        x: HUB.RED_X,
        y: HUB.RED_Y,
        size: HUB.SIZE,
        alliance: 'red',
        active: state.match.redHubActive
    });
    objects.push({
        type: 'hub',
        x: HUB.BLUE_X,
        y: HUB.BLUE_Y,
        size: HUB.SIZE,
        alliance: 'blue',
        active: state.match.blueHubActive
    });

    // Add TOWERs
    objects.push({ type: 'tower', x: TOWER.RED_X, y: TOWER.RED_Y, alliance: 'red' });
    objects.push({ type: 'tower', x: TOWER.BLUE_X, y: TOWER.BLUE_Y, alliance: 'blue' });

    // Add DEPOTs
    objects.push({ type: 'depot', x: DEPOT.RED_X, y: DEPOT.RED_Y, alliance: 'red' });
    objects.push({ type: 'depot', x: DEPOT.BLUE_X, y: DEPOT.BLUE_Y, alliance: 'blue' });

    // Add BUMPs
    BUMPS.forEach(bump => {
        objects.push({ type: 'bump', x: bump.x, y: bump.y, alliance: bump.alliance });
    });

    // Add AprilTags
    APRILTAGS.forEach(tag => {
        objects.push({
            type: 'apriltag',
            x: tag.x,
            y: tag.y,
            z: tag.z,
            id: tag.id,
            rotation: tag.rotation
        });
    });

    // Add FUEL on field
    if (state.fuel && state.fuel.field) {
        state.fuel.field.forEach(fuel => {
            objects.push({ type: 'fuel', x: fuel.x, y: fuel.y });
        });
    }

    // Add other robots
    if (state.multiRobotEnabled && state.allRobots) {
        state.allRobots.forEach(robot => {
            if (!robot.isPlayer) {
                objects.push({
                    type: 'robot',
                    x: robot.x,
                    y: robot.y,
                    heading: robot.heading,
                    alliance: robot.alliance,
                    teamNumber: robot.teamNumber,
                    fuelCount: robot.fuelCount
                });
            }
        });
    }

    return objects;
}

/**
 * Transform objects for a specific camera view.
 */
function transformObjectsForCamera(objects, robotX, robotY, robotHeading, fov, headingOffset) {
    const maxDist = 15;
    const cameraHeading = robotHeading + headingOffset;

    return objects.map(obj => {
        const dx = obj.x - robotX;
        const dy = obj.y - robotY;

        // Rotate by camera heading (robot heading + camera offset)
        const localX = dx * Math.cos(-cameraHeading) - dy * Math.sin(-cameraHeading);
        const localY = dx * Math.sin(-cameraHeading) + dy * Math.cos(-cameraHeading);

        const dist = Math.sqrt(localX * localX + localY * localY);
        const angle = Math.atan2(localY, localX);

        return { ...obj, localX, localY, dist, angle };
    }).filter(obj => {
        // Only render objects in front of camera and within FOV
        return obj.localX > 0.3 && Math.abs(obj.angle) < fov / 2 && obj.dist < maxDist;
    }).sort((a, b) => b.dist - a.dist);
}

/**
 * Render a single FPV camera view.
 * Returns the visible objects for this camera.
 */
function renderFPVCamera(canvas, ctx, objects, playerRobot, fov, headingOffset) {
    const w = canvas.width;
    const h = canvas.height;

    const robotX = playerRobot.x;
    const robotY = playerRobot.y;
    const robotHeading = -playerRobot.heading * Math.PI / 180;

    // Clear canvas with sky gradient
    const skyGradient = ctx.createLinearGradient(0, 0, 0, h / 2);
    skyGradient.addColorStop(0, '#1a1a2e');
    skyGradient.addColorStop(1, '#16213e');
    ctx.fillStyle = skyGradient;
    ctx.fillRect(0, 0, w, h / 2);

    // Draw ground with perspective
    const groundGradient = ctx.createLinearGradient(0, h / 2, 0, h);
    groundGradient.addColorStop(0, '#2d4a3a');
    groundGradient.addColorStop(1, '#1a2d22');
    ctx.fillStyle = groundGradient;
    ctx.fillRect(0, h / 2, w, h / 2);

    // Draw grid lines on ground
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.lineWidth = 1;

    for (let i = 1; i <= 10; i++) {
        const y = h / 2 + (h / 2) * (1 - 1 / (i * 0.5 + 1));
        const spread = 1 + (i * 0.3);
        ctx.beginPath();
        ctx.moveTo(w / 2 - w * spread / 2, y);
        ctx.lineTo(w / 2 + w * spread / 2, y);
        ctx.stroke();
    }

    for (let i = -5; i <= 5; i++) {
        ctx.beginPath();
        ctx.moveTo(w / 2 + i * 20, h / 2);
        ctx.lineTo(w / 2 + i * 100, h);
        ctx.stroke();
    }

    // Transform and filter objects for this camera
    const visibleObjects = transformObjectsForCamera(objects, robotX, robotY, robotHeading, fov, headingOffset);

    // Render objects
    visibleObjects.forEach(obj => {
        renderFPVObjectToCtx(ctx, obj, w, h, fov);
    });

    // Return visible objects for AprilTag indicator
    return visibleObjects;
}

/**
 * Legacy renderFPV for backwards compatibility.
 */
function renderFPV() {
    renderAllFPV();
}

/**
 * Render a single object in FPV perspective to a specific canvas context.
 */
function renderFPVObjectToCtx(ctx, obj, w, h, fov) {
    // Calculate screen position
    const screenX = w / 2 + (obj.angle / (fov / 2)) * (w / 2);
    const depthScale = 1 / (obj.dist * 0.15 + 0.5);
    const screenY = h / 2 + depthScale * 50; // Objects appear lower as they get closer

    switch (obj.type) {
        case 'hub':
            drawFPVHub(ctx, screenX, screenY, depthScale, obj);
            break;
        case 'tower':
            drawFPVTower(ctx, screenX, screenY, depthScale, obj);
            break;
        case 'depot':
            drawFPVDepot(ctx, screenX, screenY, depthScale, obj);
            break;
        case 'bump':
            drawFPVBump(ctx, screenX, screenY, depthScale, obj);
            break;
        case 'fuel':
            drawFPVFuel(ctx, screenX, screenY, depthScale);
            break;
        case 'robot':
            drawFPVRobot(ctx, screenX, screenY, depthScale, obj);
            break;
        case 'apriltag':
            drawFPVAprilTag(ctx, screenX, screenY, depthScale, obj);
            break;
    }
}

/**
 * Render a single object in FPV perspective (legacy, uses global fpvCtx).
 */
function renderFPVObject(obj, w, h, fov) {
    renderFPVObjectToCtx(fpvCtx, obj, w, h, fov);
}

/**
 * Draw a HUB in FPV.
 */
function drawFPVHub(ctx, x, y, scale, obj) {
    const size = 80 * scale;
    const color = obj.alliance === 'red' ? '#e94560' : '#3498db';

    // HUB structure
    if (obj.active) {
        ctx.shadowColor = color;
        ctx.shadowBlur = 20 * scale;
    }

    ctx.fillStyle = obj.active ? color : '#444';
    ctx.strokeStyle = obj.active ? '#fff' : '#666';
    ctx.lineWidth = 3 * scale;

    // Draw 3D box shape
    ctx.beginPath();
    ctx.moveTo(x - size / 2, y);
    ctx.lineTo(x - size / 2, y - size);
    ctx.lineTo(x + size / 2, y - size);
    ctx.lineTo(x + size / 2, y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Top face
    ctx.fillStyle = obj.active ? lightenColor(color, 20) : '#555';
    ctx.beginPath();
    ctx.moveTo(x - size / 2, y - size);
    ctx.lineTo(x - size / 3, y - size - size / 4);
    ctx.lineTo(x + size / 3, y - size - size / 4);
    ctx.lineTo(x + size / 2, y - size);
    ctx.closePath();
    ctx.fill();

    ctx.shadowBlur = 0;

    // Label
    if (scale > 0.3) {
        ctx.fillStyle = '#fff';
        ctx.font = `bold ${Math.max(12, 18 * scale)}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.fillText('HUB', x, y - size / 2);
    }
}

/**
 * Draw a TOWER in FPV.
 */
function drawFPVTower(ctx, x, y, scale, obj) {
    const width = 50 * scale;
    const height = 100 * scale;
    const color = obj.alliance === 'red' ? '#c0392b' : '#2980b9';

    ctx.fillStyle = color;
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2 * scale;

    // Tower body
    ctx.fillRect(x - width / 2, y - height, width, height);
    ctx.strokeRect(x - width / 2, y - height, width, height);

    // Label
    if (scale > 0.25) {
        ctx.fillStyle = '#fff';
        ctx.font = `${Math.max(10, 14 * scale)}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.fillText('TOWER', x, y - height / 2);
    }
}

/**
 * Draw a DEPOT in FPV.
 */
function drawFPVDepot(ctx, x, y, scale, obj) {
    const width = 40 * scale;
    const height = 30 * scale;
    const color = obj.alliance === 'red' ? 'rgba(231, 76, 60, 0.7)' : 'rgba(52, 152, 219, 0.7)';

    ctx.fillStyle = color;
    ctx.strokeStyle = obj.alliance === 'red' ? '#e74c3c' : '#3498db';
    ctx.lineWidth = 2 * scale;

    ctx.fillRect(x - width / 2, y - height, width, height);
    ctx.strokeRect(x - width / 2, y - height, width, height);
}

/**
 * Draw a BUMP in FPV.
 */
function drawFPVBump(ctx, x, y, scale, obj) {
    const width = 60 * scale;
    const height = 25 * scale;
    const color = obj.alliance === 'red' ? 'rgba(255, 180, 100, 0.7)' : 'rgba(100, 180, 255, 0.7)';

    ctx.fillStyle = color;
    ctx.strokeStyle = obj.alliance === 'red' ? '#ffa500' : '#5dade2';
    ctx.lineWidth = 2 * scale;

    // Draw bump as a raised platform shape
    ctx.beginPath();
    ctx.moveTo(x - width / 2, y);
    ctx.lineTo(x - width / 2 + 5 * scale, y - height);
    ctx.lineTo(x + width / 2 - 5 * scale, y - height);
    ctx.lineTo(x + width / 2, y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
}

/**
 * Draw FUEL in FPV.
 */
function drawFPVFuel(ctx, x, y, scale) {
    const radius = Math.max(4, 15 * scale);

    ctx.fillStyle = '#f77f00';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.arc(x, y - radius, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Highlight
    ctx.fillStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.beginPath();
    ctx.arc(x - radius * 0.3, y - radius - radius * 0.3, radius * 0.3, 0, Math.PI * 2);
    ctx.fill();
}

/**
 * Draw another robot in FPV.
 */
function drawFPVRobot(ctx, x, y, scale, obj) {
    const width = 50 * scale;
    const height = 60 * scale;

    // Robot body color
    const color = obj.alliance === 'RED' ? '#c0392b' : '#2980b9';

    ctx.fillStyle = color;
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2 * scale;

    // Robot body
    ctx.fillRect(x - width / 2, y - height, width, height);
    ctx.strokeRect(x - width / 2, y - height, width, height);

    // Team number
    if (scale > 0.2) {
        ctx.fillStyle = '#fff';
        ctx.font = `bold ${Math.max(10, 16 * scale)}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(obj.teamNumber.toString(), x, y - height / 2);
    }

    // FUEL count indicator
    if (obj.fuelCount > 0 && scale > 0.25) {
        ctx.fillStyle = '#f77f00';
        ctx.font = `bold ${Math.max(8, 12 * scale)}px sans-serif`;
        ctx.fillText(`${obj.fuelCount}`, x, y - height / 4);
    }
}

/**
 * Draw an AprilTag in FPV.
 * AprilTags are rendered as distinctive square markers with ID numbers.
 */
function drawFPVAprilTag(ctx, x, y, scale, obj) {
    const size = 40 * scale;
    const borderSize = size * 0.15;

    // Only render if large enough to be visible
    if (scale < 0.1) return;

    // Adjust Y position based on tag height (z coordinate)
    // Higher tags appear higher in the view
    const heightOffset = (obj.z - 0.5) * 30 * scale;
    const tagY = y - heightOffset;

    // Draw outer black border
    ctx.fillStyle = '#000';
    ctx.fillRect(x - size/2, tagY - size, size, size);

    // Draw white border inside
    ctx.fillStyle = '#fff';
    ctx.fillRect(
        x - size/2 + borderSize,
        tagY - size + borderSize,
        size - borderSize * 2,
        size - borderSize * 2
    );

    // Draw inner black area (simplified pattern)
    ctx.fillStyle = '#000';
    const innerSize = size - borderSize * 4;
    ctx.fillRect(
        x - innerSize/2,
        tagY - size + borderSize * 2,
        innerSize,
        innerSize
    );

    // Draw tag ID number in white
    if (scale > 0.15) {
        ctx.fillStyle = '#fff';
        ctx.font = `bold ${Math.max(10, 16 * scale)}px monospace`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(obj.id.toString(), x, tagY - size/2);
    }

    // Draw "AT" label below tag for clarity
    if (scale > 0.2) {
        ctx.fillStyle = '#0f0';
        ctx.font = `${Math.max(8, 10 * scale)}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.fillText(`TAG ${obj.id}`, x, tagY + 5);
    }
}

/**
 * Update the FPV HUD elements.
 */
function updateFPVHUD(playerRobot, visibleObjects) {
    const headingEl = document.getElementById('fpv-heading');
    const fuelEl = document.getElementById('fpv-fuel');
    const targetEl = document.getElementById('fpv-target');

    if (headingEl) {
        headingEl.textContent = `HDG: ${Math.round(playerRobot.heading)}°`;
    }

    if (fuelEl) {
        const fuelCount = playerRobot.fuelCount || 0;
        fuelEl.textContent = `FUEL: ${fuelCount}`;
    }

    if (targetEl) {
        // Find nearest target (hub or fuel)
        const nearestHub = visibleObjects.find(obj => obj.type === 'hub' && obj.active);
        const nearestFuel = visibleObjects.find(obj => obj.type === 'fuel');

        if (nearestHub && nearestHub.dist < 8) {
            targetEl.textContent = `TARGET: ${nearestHub.alliance.toUpperCase()} HUB ${nearestHub.dist.toFixed(1)}m`;
            targetEl.style.color = nearestHub.alliance === 'red' ? '#e94560' : '#3498db';
        } else if (nearestFuel) {
            targetEl.textContent = `TARGET: FUEL ${nearestFuel.dist.toFixed(1)}m`;
            targetEl.style.color = '#f77f00';
        } else {
            targetEl.textContent = 'TARGET: ---';
            targetEl.style.color = '#8b949e';
        }
    }
}

/**
 * Update the AprilTag visibility indicator for a camera.
 */
function updateAprilTagIndicator(elementId, visibleObjects) {
    const container = document.getElementById(elementId);
    if (!container) return;

    const listEl = container.querySelector('.apriltag-list');
    if (!listEl) return;

    // Filter for AprilTags only
    const visibleTags = visibleObjects
        .filter(obj => obj.type === 'apriltag')
        .sort((a, b) => a.id - b.id);

    if (visibleTags.length === 0) {
        listEl.innerHTML = '---';
        listEl.className = 'apriltag-list empty';
    } else {
        // Generate HTML for each visible tag with color coding
        const tagHtml = visibleTags.map(tag => {
            let tagClass = 'apriltag-id';
            // Color code based on tag ID:
            // 1-6: HUB tags (blue)
            // 7-10: TOWER tags (purple)
            // 11-16: WALL tags (gray)
            if (tag.id >= 1 && tag.id <= 6) {
                tagClass += ' hub-tag';
            } else if (tag.id >= 7 && tag.id <= 10) {
                tagClass += ' tower-tag';
            } else {
                tagClass += ' wall-tag';
            }
            return `<span class="${tagClass}">${tag.id}</span>`;
        }).join('');

        listEl.innerHTML = tagHtml;
        listEl.className = 'apriltag-list';
    }
}

/**
 * Helper to lighten a color.
 */
function lightenColor(color, percent) {
    const num = parseInt(color.replace('#', ''), 16);
    const amt = Math.round(2.55 * percent);
    const R = (num >> 16) + amt;
    const G = (num >> 8 & 0x00FF) + amt;
    const B = (num & 0x0000FF) + amt;
    return '#' + (
        0x1000000 +
        (R < 255 ? (R < 1 ? 0 : R) : 255) * 0x10000 +
        (G < 255 ? (G < 1 ? 0 : G) : 255) * 0x100 +
        (B < 255 ? (B < 1 ? 0 : B) : 255)
    ).toString(16).slice(1);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

function init() {
    initCanvas();
    initFPVCanvas();
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
