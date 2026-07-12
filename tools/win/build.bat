@echo off
REM Build script for the Windows CV1 capture tools.
REM
REM Builds against the vendored LibOVR mirror (D:\third_party\LibOVR, SDK 1.40,
REM from https://github.com/opentrack/LibOVR branch-140) by compiling
REM OVR_CAPIShim.c directly — no prebuilt LibOVR.lib and no OVR_SDK download
REM needed. The shim dynamically loads the installed LibOVRRT64_1.dll from the
REM Meta Horizon runtime at run time.
REM
REM Set OVR_SDK to override the SDK location (must contain LibOVR\Include and
REM LibOVR\Src\OVR_CAPIShim.c).
REM
REM Run from an "x64 Native Tools" prompt, or let this script find VsDevCmd.

if "%OVR_SDK%"=="" set OVR_SDK=%~dp0..\..\third_party\LibOVR

set OVR_INC=%OVR_SDK%\LibOVR\Include
set OVR_SHIM=%OVR_SDK%\LibOVR\Src\OVR_CAPIShim.c

if not exist "%OVR_SHIM%" (
    echo ERROR: %OVR_SHIM% not found.
    echo   git clone --depth 1 -b branch-140 https://github.com/opentrack/LibOVR.git ..\..\third_party\LibOVR
    exit /b 1
)

where cl >nul 2>&1
if not %ERRORLEVEL%==0 (
    echo cl not on PATH — loading VS dev environment...
    call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -no_logo
    where cl >nul 2>&1 || ( echo ERROR: MSVC not found. & exit /b 1 )
)

echo Building ovr_static_dump.exe ...
cl /nologo /W3 /O2 /D_CRT_SECURE_NO_WARNINGS /I "%OVR_INC%" ^
   ovr_static_dump.c "%OVR_SHIM%" advapi32.lib /Fe:ovr_static_dump.exe
if %ERRORLEVEL% neq 0 ( echo FAILED & exit /b 1 )

echo Building ovr_pose_log.exe ...
cl /nologo /W3 /O2 /D_CRT_SECURE_NO_WARNINGS /I "%OVR_INC%" ^
   ovr_pose_log.c "%OVR_SHIM%" advapi32.lib winmm.lib /Fe:ovr_pose_log.exe
if %ERRORLEVEL% neq 0 ( echo FAILED & exit /b 1 )

echo.
echo Build successful:
echo   ovr_static_dump.exe  [outfile.json] [wait_s]
echo   ovr_pose_log.exe     ^<out_now.csv^> ^<out_pred.csv^> [duration_s]
