// ROS Bridge MOD for Arma 3
// Enables communication between Arma 3 and ROS via TCP/IP using ArmaCOM

class CfgPatches
{
    class ros_bridge
    {
        name = "ROS Bridge";
        author = "HitTheStars";
        url = "https://github.com/HitTheStars/Ros_arma3_Connection";
        
        units[] = {};
        weapons[] = {};
        requiredVersion = 2.00;
        requiredAddons[] = {};
        
        version = "1.0";
        versionStr = "1.0.0";
        versionAr[] = {1,0,0};
    };
};

class CfgFunctions
{
    class ROS
    {
        tag = "ROS";
        
        class Communication
        {
            file = "\ros_bridge\functions\communication";
            class initBridge {};
            class connectToServer {};
            class sendImage {};
            class sendStatus {};
            class receiveCommand {};
            class executeCommand {};
        };
        
        class UAV
        {
            file = "\ros_bridge\functions\uav";
            class initUAVs {};
            class setupCameras {};
            class captureImages {};
            class getUAVStatus {};
            class controlUAV {};
        };
        
        class Utils
        {
            file = "\ros_bridge\functions\utils";
            class arrayToString {};
            class stringToArray {};
            class logMessage {};
        };
    };
};

class Extended_PreInit_EventHandlers
{
    class ros_bridge_preinit
    {
        init = "call compile preprocessFileLineNumbers '\ros_bridge\XEH_preInit.sqf'";
    };
};

class Extended_PostInit_EventHandlers
{
    class ros_bridge_postinit
    {
        init = "call compile preprocessFileLineNumbers '\ros_bridge\XEH_postInit.sqf'";
    };
};
