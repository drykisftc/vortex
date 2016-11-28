@echo off
REM Use this Batch file to wirelessly conenct to an Android Phone for code development
REM This assumes that you have previously connected your phone to the PC  
REM If phone has shown up as "Direct Connection" instead of "Access Point", try unpairing,
REM and follow these instructions EXACTLY:https://productforums.google.com/forum/#!topic/nexus/bivQfyYPPD0
REM *****  For more help, go to www.YouTube.com/user/GEARSinc/playlists
REM More info may be posted on FTC forums
REM Original by Philbot from GEARSinc, modified by Bendito999 for Bluetooth
REM

echo --  Starting ADB
adb kill-server
echo --  Turn on "Bluetooth Tethering" in phone settings (it defaults off)
echo --  Make sure the phone is paired to the PC Bluetooth.
echo --  Make sure that phone is connected as "Access Point" from Devices and Printers
echo --  This can be found by right clicking Bluetooth icon
echo --  then pressing "Join a Personal Area Network";right click on device 
echo --  ALSO, make sure the phone is connected to the computer via USB
REM We could also auto open devices and printers from here
REM explorer shell:::{A8A91A66-3A7D-4424-8D24-04E180695C7A}
set /p ok= --  Hit enter when phone has been plugged in and recognized: 
echo -- Checking ping for debugging purposes
echo -- If ping fails ensure the previous bluetooth instructions were followed
echo -- Also see this link for more help with bluetooth setup
echo -- https://productforums.google.com/forum/#!topic/nexus/bivQfyYPPD0
REM alternate link to google doc echo -- https://docs.google.com/document/d/1F_YLDfGBVJDL9lTZ5nbm4B6sTX8HcufbuIswQ_MZxk8/edit?usp=sharing
ping 192.168.44.1
adb usb
Timeout 10
adb tcpip 4455
Timeout 5
adb connect 192.168.44.1:4455
adb devices
set /p ok= --  Unplug the phone and hit Enter to see the final connection.
adb connect 192.168.44.1:4455
Timeout 5
adb devices
Timeout 5

