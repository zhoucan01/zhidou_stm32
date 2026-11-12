@echo off
setlocal EnableExtensions DisableDelayedExpansion
REM Drag firmware_full.bin / firmware_full.hex onto this file, or pass its path.
REM Programs the selected full image, never the APP project's AXF.
if defined PYOCD_PY goto run
where python.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not found. Set PYOCD_PY to your Python executable.
    pause
    exit /b 1
)
set "PYOCD_PY=python"
:run
"%PYOCD_PY%" "%~dp0AT32_OTA_PC\flash_merged.py" %*
set "FLASH_RC=%ERRORLEVEL%"
if not "%FLASH_RC%"=="0" echo [FAILED] Download did not complete. See the message above.
pause
exit /b %FLASH_RC%
