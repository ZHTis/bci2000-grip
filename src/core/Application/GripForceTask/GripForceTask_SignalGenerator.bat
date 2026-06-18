@echo off
REM GripForceTask_SignalGenerator.bat
REM 用SignalGenerator仿真握力（鼠标Y轴=握力）

cd /d "%~dp0\.."

START "" "%~dp0\..\prog\Operator.exe" --Title "Grip Force Task"
TIMEOUT /T 1 /NOBREAK > NUL

START "" "%~dp0\..\prog\SignalGenerator.exe" 127.0.0.1 4000 4001
START "" "%~dp0\..\prog\SpectralSignalProcessing.exe" 127.0.0.1 4001 4002
START "" "%~dp0\..\prog\GripForceTask.exe" 127.0.0.1 4002 4003

REM 等待模块启动
TIMEOUT /T 2 /NOBREAK > NUL

REM 自动加载参数文件
REM "%~dp0\..\prog\Operator.exe" --load-parameters "%~dp0\..\parms\examples\GripForceTask_SignalGenerator.prm"
