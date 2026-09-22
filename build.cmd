@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo ERROR: vswhere.exe was not found. Install Visual Studio with Desktop development with C++.
    exit /b 1
)

set "MSBUILD="
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe`) do if not defined MSBUILD set "MSBUILD=%%I"
if not defined MSBUILD (
    echo ERROR: MSBuild was not found.
    exit /b 1
)

"%MSBUILD%" open1V.slnx /m /p:Configuration=Release /p:Platform=x64
if errorlevel 1 exit /b 1

echo Built open1V.exe, open1V-debug.exe, and open1v-core.lib
