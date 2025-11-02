// Arma 3 Mission Initialization Script
// This script runs when the mission starts
// It sets up UAVs and prepares for ROS communication

// ============================================
// Configuration
// ============================================

// Number of UAVs
_uavCount = 3;

// UAV type (can be changed to other UAV models)
_uavType = "B_UAV_02_dynamicLoadout_F"; // MQ-4A Greyhawk

// Starting positions (relative to mission center)
_startPositions = [
    [0, 0, 100],
    [50, 0, 100],
    [-50, 0, 100]
];

// ============================================
// Create UAVs
// ============================================

systemChat "Initializing UAVs...";

// Array to store UAV objects
uavArray = [];

for "_i" from 0 to (_uavCount - 1) do {
    _pos = _startPositions select _i;
    _worldPos = [getPos player select 0 + (_pos select 0), getPos player select 1 + (_pos select 1), _pos select 2];
    
    // Create UAV
    _uav = createVehicle [_uavType, _worldPos, [], 0, "FLY"];
    _uav setVehicleVarName format ["uav_%1", _i];
    
    // Set UAV properties
    _uav engineOn true;
    _uav flyInHeight 100;
    
    // Add to array
    uavArray pushBack _uav;
    
    systemChat format ["UAV %1 created at %2", _i, _worldPos];
};

// ============================================
// Communication Functions
// ============================================

// Function to get UAV status
fnc_getUAVStatus = {
    params ["_uavIndex"];
    
    _uav = uavArray select _uavIndex;
    
    _pos = getPosASL _uav;
    _vel = velocity _uav;
    _dir = getDir _uav;
    
    // Return status as array
    [_pos, _vel, _dir]
};

// Function to move UAV to target position
fnc_moveUAV = {
    params ["_uavIndex", "_targetPos"];
    
    _uav = uavArray select _uavIndex;
    
    // Create waypoint
    _group = group _uav;
    _wp = _group addWaypoint [_targetPos, 0];
    _wp setWaypointType "MOVE";
    _wp setWaypointSpeed "NORMAL";
    
    systemChat format ["UAV %1 moving to %2", _uavIndex, _targetPos];
};

// Function to stop UAV
fnc_stopUAV = {
    params ["_uavIndex"];
    
    _uav = uavArray select _uavIndex;
    
    // Remove all waypoints
    _group = group _uav;
    while {(count (waypoints _group)) > 0} do {
        deleteWaypoint ((waypoints _group) select 0);
    };
    
    // Set velocity to zero
    _uav setVelocity [0, 0, 0];
    
    systemChat format ["UAV %1 stopped", _uavIndex];
};

// ============================================
// Camera Setup for Image Capture
// ============================================

// Create cameras for each UAV
cameraArray = [];

for "_i" from 0 to (_uavCount - 1) do {
    _uav = uavArray select _i;
    
    // Create camera attached to UAV
    _camera = "camera" camCreate [0,0,0];
    _camera cameraEffect ["Internal", "Back"];
    _camera attachTo [_uav, [0, 0, -2]];
    _camera camSetFov 0.7;
    
    cameraArray pushBack _camera;
};

// ============================================
// Main Loop (Status Update)
// ============================================

[] spawn {
    while {true} do {
        // Update UAV status every second
        {
            _status = [_forEachIndex] call fnc_getUAVStatus;
            // Status will be read by external program (Python bridge)
            // Store in global variable for access
            missionNamespace setVariable [format ["uav_%1_status", _forEachIndex], _status];
        } forEach uavArray;
        
        sleep 1;
    };
};

// ============================================
// Initialization Complete
// ============================================

systemChat "UAV initialization complete!";
systemChat format ["Total UAVs: %1", count uavArray];

// Display instructions
hint "UAV Mission Ready!\n\nUAVs are now controllable via ROS.\n\nPress ESC to access mission menu.";
