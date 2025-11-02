// ROS Bridge Post-Init Script
// This script initializes the ROS bridge after the mission starts

diag_log "[ROS Bridge] Post-Init started";

// Global variables
ROS_Connected = false;
ROS_ClientUUID = "";
ROS_ServerIP = "192.168.1.100";  // Change this to your Linux VM IP
ROS_ServerPort = "5555";
ROS_UAVs = [];
ROS_Cameras = [];
ROS_LastImageTime = 0;
ROS_LastStatusTime = 0;
ROS_ImageInterval = 0.5;  // Send images every 0.5 seconds (2 Hz)
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
                
                // Parse command (format: "MOVE:X,Y,Z" or "GOAL:X,Y,Z")
                private _parts = _message splitString ":";
                if (count _parts >= 2) then {
                    private _cmd = _parts select 0;
                    private _args = (_parts select 1) splitString ",";
                    
                    if (_cmd == "MOVE" && count _args >= 3) then {
                        private _x = parseNumber (_args select 0);
                        private _y = parseNumber (_args select 1);
                        private _z = parseNumber (_args select 2);
                        
                        // Move all UAVs to the target position
                        {
                            _x doMove [_x, _y, _z];
                            diag_log format ["[ROS Bridge] Moving UAV %1 to [%2, %3, %4]", _x, _x, _y, _z];
                        } forEach ROS_UAVs;
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

// Function to send camera images to ROS (placeholder - actual image data would be sent by Python bridge)
ROS_fnc_sendImageNotification = {
    if (!ROS_Connected) exitWith {};
    
    // Send notification that images are ready
    private _message = format ["IMAGE_READY:%1\n", time];
    "ArmaCOM" callExtension [ROS_ClientUUID, ["write", _message]];
};

// Main loop
ROS_fnc_mainLoop = {
    while {ROS_Connected} do {
        private _currentTime = time;
        
        // Send status at regular intervals
        if (_currentTime - ROS_LastStatusTime >= ROS_StatusInterval) then {
            [] call ROS_fnc_sendStatus;
            ROS_LastStatusTime = _currentTime;
        };
        
        // Send image notification at regular intervals
        if (_currentTime - ROS_LastImageTime >= ROS_ImageInterval) then {
            [] call ROS_fnc_sendImageNotification;
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

diag_log "[ROS Bridge] Post-Init completed";
