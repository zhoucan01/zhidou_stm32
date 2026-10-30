@echo off
setlocal

REM ==================== Config ====================
REM KEIL_PATH can be set in environment or left as default.
if not defined KEIL_PATH (
    set "KEIL_PATH=C:\Keil_v5"
)

REM Try common alternate installation paths.
if not exist "%KEIL_PATH%\UV4\UV4.exe" (
    if exist "D:\application\KEIL\ARM\UV4\UV4.exe" (
        set "KEIL_PATH=D:\application\KEIL\ARM"
    )
)

set "UV4=%KEIL_PATH%\UV4\UV4.exe"
set "PROJECT_DIR=%~dp0"
set "PROJECT_ROOT=%PROJECT_DIR%project\MDK_V5"
set "UVPROJX=%PROJECT_ROOT%\code.uvprojx"
set "OUTPUT_DIR=%PROJECT_ROOT%\objects"
set "LOG_FILE=%PROJECT_ROOT%\build_log.txt"
REM =================================================

REM Check Keil
if not exist "%UV4%" (
    echo [ERROR] UV4.exe not found: %UV4%
    echo         Please check KEIL_PATH in this script or install Keil MDK.
    exit /b 1
)

REM Check project file
if not exist "%UVPROJX%" (
    echo [ERROR] Project file not found: %UVPROJX%
    exit /b 1
)

REM Save all workspace files in VS Code before building.
REM NOTE: VS Code CLI does NOT support a '--command' option, so we cannot
REM trigger workbench.action.files.saveAll from outside. Instead, saving is
REM handled by the VS Code Task 'saveAndBuild' (see .vscode/tasks.json) which
REM runs workbench.action.files.saveAll before invoking this script.
REM If you run build.bat directly from a terminal, make sure to save files
REM manually in VS Code first (Ctrl+K S).

REM Ensure output dir exists
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

REM ---- Parse args ----
if /i "%~1"=="-Clean" goto :clean
if /i "%~1"=="clean" goto :clean
if /i "%~1"=="-Rebuild" goto :rebuild
if /i "%~1"=="rebuild" goto :rebuild

REM ---- Incremental build ----
:build
set "ACTION=-b"
set "ACTION_TEXT=Build"
goto :dobuild

REM ---- Rebuild all ----
:rebuild
set "ACTION=-r"
set "ACTION_TEXT=Rebuild"

:dobuild
echo ==== AT32F415RBT7 %ACTION_TEXT% ====
echo Project: %UVPROJX%
echo Tool:    %UV4%
echo Building...

"%UV4%" %ACTION% "%UVPROJX%" -j0 -o "%LOG_FILE%"
set "EXIT_CODE=%ERRORLEVEL%"

echo.
echo ==== Build Log ====
if exist "%LOG_FILE%" type "%LOG_FILE%"

echo.
echo ==== Build Result ====
REM UV4 exit codes: 0=OK, 1=warnings, others=error
if "%EXIT_CODE%"=="0" echo [SUCCESS] Build OK, no errors, no warnings
if "%EXIT_CODE%"=="1" echo [WARNING] Build OK with warnings. Exit code: 1
if not "%EXIT_CODE%"=="0" if not "%EXIT_CODE%"=="1" (
    echo [FAILED] Build failed. Exit code: %EXIT_CODE%
    exit /b %EXIT_CODE%
)

REM Check target output files
set "HEX_FILE=%OUTPUT_DIR%\code.hex"
set "AXF_FILE=%OUTPUT_DIR%\code.axf"
if exist "%HEX_FILE%" echo HEX: %HEX_FILE%
if exist "%AXF_FILE%" echo AXF: %AXF_FILE%

REM Analyze flash/RAM usage from the linker map file
set "MAP_FILE=%PROJECT_ROOT%\listings\code.map"
set "ANALYZE_PS1=%PROJECT_DIR%analyze_map.ps1"
if exist "%MAP_FILE%" (
    if exist "%ANALYZE_PS1%" (
        echo.
        powershell -NoProfile -ExecutionPolicy Bypass -File "%ANALYZE_PS1%" -MapFile "%MAP_FILE%"
    ) else (
        echo [WARN] analyze_map.ps1 not found, skipping map analysis
    )
) else (
    echo [WARN] Map file not found: %MAP_FILE%
)

exit /b %EXIT_CODE%

:clean
echo ==== Clean Build Output ====
if exist "%OUTPUT_DIR%" (
    del /q /f "%OUTPUT_DIR%\*.o" 2>nul
    del /q /f "%OUTPUT_DIR%\*.d" 2>nul
    del /q /f "%OUTPUT_DIR%\*.crf" 2>nul
    del /q /f "%OUTPUT_DIR%\*.axf" 2>nul
    del /q /f "%OUTPUT_DIR%\*.hex" 2>nul
    del /q /f "%OUTPUT_DIR%\*.htm" 2>nul
    del /q /f "%OUTPUT_DIR%\*.map" 2>nul
    del /q /f "%OUTPUT_DIR%\*.lnp" 2>nul
    del /q /f "%OUTPUT_DIR%\*.sct" 2>nul
    del /q /f "%OUTPUT_DIR%\*.dep" 2>nul
)
if exist "%PROJECT_ROOT%\build_log.txt" del /q /f "%PROJECT_ROOT%\build_log.txt" 2>nul
if exist "%PROJECT_ROOT%\listings" del /q /f "%PROJECT_ROOT%\listings\*.lst" 2>nul
echo [OK] Clean done
exit /b 0
