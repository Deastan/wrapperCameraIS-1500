isense Unity Plugin Readme 05/10/2017
------------------
isense.unitypackage was built with Unity v4.7.2f1. It contains the following:
-32 and 64 bit versions of the isense library, for Windows and Linux. These are the same as the ones in the SDKs/isenseSDK folders, with libisense.so renamed to isense.so for compatibility purposes
-ISenseLib.cs, which is required for the library to work with C# scripts
-ISenseSample.cs, a sample script to be attached to a Unity object which controls rotation and position based on those of the tracker
-AngleUtil.cs, which is required for the sample script to handle rotation properly
-An iSenseCam prefab, which is a basic Unity camera with the ISenseSample script attached
-A scene file, and related scripts, which can be used to build isUnitySample
-Some example files for testing with an augmented reality setup.

Only the isense library files and ISenseLib.cs are required for developing Unity applications for InterSense hardware.
ISenseSample.cs can be used as a reference for how to communicate with InterSense devices.

Note that to use the library, you must use isense.ini and sfaccess.ini files. Examples of these may be found in ConfigurationFiles/sfAccessExampleConfiguration in the IS1500 package. For NFT use, set UdpSfCorePort to 0.
For use in the Unity Editor, these files are best placed in C:/Dev on Windows and /dev/ on Linux. Once the project is built, they may be placed in the same folder as the project's executable.

The iSenseCam prefab and scene file were made with Unity 4.7, and as such are not compatible with prior versions of Unity.
The isense library, ISenseLib.cs, and ISenseSample.cs, were tested with Unity 5.5 and no issues were found.

If a compatibility version of this package is needed, please contact techsupport@thalesvisionix.com with your Unity version number.

Updates 
05/25/17
- Updated to DLL 4.27.13

05/10/17
- General refactoring of code outside iSenseSample
- Updated to DLL 4.27.11
- Soft Reset now restarts streaming rather than restarting sfHub
- Duplicate UI buttons removed
- UI space no longer wasted to launch our website

04/24/17
- Added commands for GPS to ISenseLib
- Added IS-1500 video feed
- Rotation conversion redone, should be less finnicky. Note that boresights will have to be reevaluated (if you previously did a 90 degree pitch boresight, you will need to do a 180 degree roll boresight in addition)
- Removed 'floor' from skybox view
- Lights have been moved