set EPICS_CA_AUTO_ADDR_LIST=NO
set EPICS_CA_ADDR_LIST=192.168.1.255
set EPICS_CA_MAX_ARRAY_BYTES=10000000
start "" "C:\Program Files\ImageJ\ImageJ.exe"
set EPICS_DISPLAY_PATH=D:\Development\Clients\XCAM\EPICS\synApps\synApps_5_8\support\areaDetector-R2-4\ADCore\ADApp\op\adl;D:\Development\Clients\XCAM\EPICS\synApps\synApps_5_8\support\areaDetector-R2-4\ADxcam\exampleApp\op\adl
start medm -x -macro "P=13SIM1:, R=cam1:" xcamCamera.adl
..\..\bin\windows-x64\xcamCameraApp st.cmd.windows.phil




