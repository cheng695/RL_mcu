@echo off
setlocal

if not defined JLINK_EXE (
    for /f "delims=" %%I in ('where JLink.exe 2^>nul') do if not defined JLINK_EXE set "JLINK_EXE=%%I"
)
if not defined JLINK_EXE if exist "%ProgramFiles%\SEGGER\JLink\JLink.exe" set "JLINK_EXE=%ProgramFiles%\SEGGER\JLink\JLink.exe"
if not defined JLINK_EXE if exist "%ProgramFiles(x86)%\SEGGER\JLink\JLink.exe" set "JLINK_EXE=%ProgramFiles(x86)%\SEGGER\JLink\JLink.exe"
if not defined JLINK_EXE if exist "%ProgramFiles%\SEGGER\JLink_V892\JLink.exe" set "JLINK_EXE=%ProgramFiles%\SEGGER\JLink_V892\JLink.exe"
if not defined JLINK_EXE for %%I in ("%ProgramFiles%\SEGGER\JLink_*\JLink.exe") do if exist "%%~fI" set "JLINK_EXE=%%~fI"
if not defined JLINK_EXE for %%I in ("%ProgramFiles(x86)%\SEGGER\JLink_*\JLink.exe") do if exist "%%~fI" set "JLINK_EXE=%%~fI"

if not defined JLINK_EXE if /i not "%~1"=="--dry-run" (
    echo JLink.exe was not found. Add it to PATH or set JLINK_EXE to its full path.
    exit /b 1
)

if defined JLINK_EXE for /f "delims=" %%P in ('cygpath -u "%JLINK_EXE%" 2^>nul') do set "JLINK_EXE=%%P"

pushd "%~dp0.."
bash scripts/flash.sh %*
set "flash_exit_code=%ERRORLEVEL%"
popd
exit /b %flash_exit_code%
