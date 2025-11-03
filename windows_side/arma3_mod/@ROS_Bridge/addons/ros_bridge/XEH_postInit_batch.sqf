// ROS Bridge Post-Init Script (Batch Processing Version)
// 批处理版本的 ROS 桥接初始化脚本
//
// 本脚本添加了图像采集功能，配合批处理深度估计服务使用

diag_log "[ROS Bridge] Post-Init started (Batch Processing Version)";

// Global variables
ROS_Connected = false;
ROS_ClientUUID = "";
ROS_ServerIP = "192.168.1.100";  // Change this to your Linux VM IP
ROS_ServerPort = "5556";  // Control bridge port
ROS_UAVs = [];
ROS_Cameras = [];
ROS_LastImageTime = 0;
ROS_LastStatusTime = 0;
ROS_ImageInterval = 0.2;  // Send images every 0.2 seconds (5 Hz)
ROS_StatusInterval = 0.1;  // Send status every 0.1 seconds (10 Hz)

// Function to initialize the TCP client
ROS_fnc_initBridge = {
    diag_log "[ROS Bridge] Initializing TCP client...";
    
    // Create TCP client instance
    private _result = "ArmaCOM" callExtension ["TCPClient", ["create", ROS_ServerIP, ROS_ServerPort]];
    private _resultArray = parseSimpleArray _result;
    
    if ((_resultArray select 0) == "SUCCESS") then {
        ROS_ClientUUID = _resultArray select 1;
        diag_log format ["[ROS Bridge] TCP client created with UUID: %1", ROS_ClientUUID];
        
        // Set callback on newline character
        "ArmaCOM" callExtension [ROS_ClientUUID, ["callbackOnCharCode", "10"]];  // 10 = \n
        
        // Connect asynchronously
        "ArmaCOM" callExtension [ROS_ClientUUID, ["connectAsync"]];
        diag_log format ["[ROS Bridge] Connecting to %1:%2...", ROS_ServerIP, ROS_ServerPort];
    } else {
        diag_log format ["[ROS Bridge] ERROR: Failed to create TCP client: %1", _result];
    };
};

// Function to handle incoming data from ROS
ROS_fnc_handleCallback = {
    params ["_name", "_function", "_data"];
    
    if (_name == "ArmaCOM") then {
        if (_function == "data_read") then {
            private _dataArray = parseSimpleArray _data;
            private _uuid = _dataArray select 0;
            private _message = _dataArray select 1;
            
            if (_uuid == ROS_ClientUUID) then {
                diag_log format ["[ROS Bridge] Received from ROS: %1", _message];
                
                // Parse command (format: "MOVE:UAV_ID,X,Y,Z,VX,VY,VZ,YAW" or "GOAL:X,Y,Z")
                private _parts = _message splitString ":";
                if (count _parts >= 2) then {
                    private _cmd = _parts select 0;
                    private _args = (_parts select 1) splitString ",";
                    
                    if (_cmd == "MOVE" && count _args >= 4) then {
                        private _uavId = parseNumber (_args select 0);
                        private _targetX = parseNumber (_args select 1);
                        private _targetY = parseNumber (_args select 2);
                        private _targetZ = parseNumber (_args select 3);
                        
                        // Parse velocity and yaw if provided (format: MOVE:UAV_ID,X,Y,Z,VX,VY,VZ,YAW)
                        private _hasVelocity = (count _args >= 8);
                        private _targetVX = if (_hasVelocity) then {parseNumber (_args select 4)} else {0};
                        private _targetVY = if (_hasVelocity) then {parseNumber (_args select 5)} else {0};
                        private _targetVZ = if (_hasVelocity) then {parseNumber (_args select 6)} else {0};
                        private _targetYaw = if (_hasVelocity) then {parseNumber (_args select 7)} else {0};
                        
                        // Move specific UAV with velocity control
                        if (_uavId < count ROS_UAVs) then {
                            private _uav = ROS_UAVs select _uavId;
                            
                            if (_hasVelocity) then {
                                // Use setVelocity for precise velocity control
                                _uav setVelocity [_targetVX, _targetVY, _targetVZ];
                                
                                // Use setVectorDirAndUp for precise orientation control
                                // Calculate direction vector from yaw angle
                                private _dirX = sin _targetYaw;
                                private _dirY = cos _targetYaw;
                                private _dirZ = 0;
                                private _upVector = [0, 0, 1];  // Standard up vector
                                _uav setVectorDirAndUp [[_dirX, _dirY, _dirZ], _upVector];
                                
                                diag_log format ["[ROS Bridge] UAV %1: pos=[%2,%3,%4], vel=[%5,%6,%7], yaw=%8", 
                                    _uavId, _targetX, _targetY, _targetZ, _targetVX, _targetVY, _targetVZ, _targetYaw];
                            } else {
                                // Fallback to doMove for position-only control
                                _uav doMove [_targetX, _targetY, _targetZ];
                                diag_log format ["[ROS Bridge] Moving UAV %1 to [%2, %3, %4] (position only)", 
                                    _uavId, _targetX, _targetY, _targetZ];
                            };
                        } else {
                            diag_log format ["[ROS Bridge] ERROR: Invalid UAV ID %1", _uavId];
                        };
                    };
                    
                    if (_cmd == "GOAL" && count _args >= 3) then {
                        private _x = parseNumber (_args select 0);
                        private _y = parseNumber (_args select 1);
                        private _z = parseNumber (_args select 2);
                        
                        diag_log format ["[ROS Bridge] Goal set to [%1, %2, %3]", _x, _y, _z];
                        // EGO-Planner will handle the path planning
                    };
                };
            };
        };
        
        if (_function == "connect_result") then {
            private _dataArray = parseSimpleArray _data;
            private _uuid = _dataArray select 0;
            private _result = _dataArray select 1;
            
            if (_uuid == ROS_ClientUUID) then {
                if (_result select 0 == "SUCCESS") then {
                    ROS_Connected = true;
                    diag_log "[ROS Bridge] Successfully connected to ROS server!";
                    
                    // Start sending data
                    [] spawn ROS_fnc_mainLoop;
                } else {
                    diag_log format ["[ROS Bridge] ERROR: Failed to connect: %1", _result];
                };
            };
        };
    };
};

// Function to send UAV status to ROS
ROS_fnc_sendStatus = {
    if (!ROS_Connected) exitWith {};
    
    private _statusData = [];
    
    {
        private _uav = _x;
        private _pos = getPosASL _uav;
        private _vel = velocity _uav;
        private _dir = direction _uav;
        
        private _uavStatus = format ["UAV%1:%2,%3,%4,%5,%6,%7,%8", 
            _forEachIndex,
            _pos select 0,
            _pos select 1,
            _pos select 2,
            _vel select 0,
            _vel select 1,
            _vel select 2,
            _dir
        ];
        
        _statusData pushBack _uavStatus;
    } forEach ROS_UAVs;
    
    private _message = format ["STATUS:%1\n", _statusData joinString "|"];
    "ArmaCOM" callExtension [ROS_ClientUUID, ["write", _message]];
};

// Function to capture images from UAV cameras
ROS_fnc_captureImages = {
    if (!ROS_Connected) exitWith {};
    
    {
        private _uav = _x;
        private _uavId = _forEachIndex;
        
        // Screenshot with UAV-specific filename
        // Format: uav0_frame.png, uav1_frame.png, etc.
        private _filename = format["uav%1_frame.png", _uavId];
        screenshot _filename;
        
        diag_log format ["[ROS Bridge] Captured image for UAV %1: %2", _uavId, _filename];
        
    } forEach ROS_UAVs;
};

// Main loop
ROS_fnc_mainLoop = {
    diag_log "[ROS Bridge] Main loop started";
    
    while {ROS_Connected} do {
        private _currentTime = time;
        
        // Send status at regular intervals (10 Hz)
        if (_currentTime - ROS_LastStatusTime >= ROS_StatusInterval) then {
            [] call ROS_fnc_sendStatus;
            ROS_LastStatusTime = _currentTime;
        };
        
        // Capture images at regular intervals (5 Hz)
        if (_currentTime - ROS_LastImageTime >= ROS_ImageInterval) then {
            [] call ROS_fnc_captureImages;
            ROS_LastImageTime = _currentTime;
        };
        
        sleep 0.01;  // Small sleep to prevent CPU overload
    };
};

// Function to initialize UAVs
ROS_fnc_initUAVs = {
    diag_log "[ROS Bridge] Initializing UAVs...";
    
    // Find all UAVs in the mission
    ROS_UAVs = allUnits select {_x isKindOf "UAV"};
    
    if (count ROS_UAVs == 0) then {
        diag_log "[ROS Bridge] WARNING: No UAVs found in mission!";
    } else {
        diag_log format ["[ROS Bridge] Found %1 UAVs", count ROS_UAVs];
        
        // Set up cameras for each UAV
        {
            private _uav = _x;
            private _camera = "camera" camCreate [0,0,0];
            _camera cameraEffect ["Internal", "Back"];
            _camera attachTo [_uav, [0, 0, 0]];
            _camera camSetFov 0.75;
            
            ROS_Cameras pushBack _camera;
            
            diag_log format ["[ROS Bridge] Camera %1 attached to UAV %2", _forEachIndex, _uav];
        } forEach ROS_UAVs;
    };
};

// Register callback handler
addMissionEventHandler ["ExtensionCallback", {
    _this call ROS_fnc_handleCallback;
}];

// Initialize everything
[] call ROS_fnc_initUAVs;
[] call ROS_fnc_initBridge;

diag_log "[ROS Bridge] Post-Init completed (Batch Processing Version)";
diag_log format ["[ROS Bridge] Image capture frequency: %1 Hz", 1 / ROS_ImageInterval];
diag_log "[ROS Bridge] Images will be saved to Screenshots folder and processed by batch service";
