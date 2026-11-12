@echo off
setlocal

REM Build the Bootloader project, not the separate APP project.
if not defined KEIL_PATH set "KEIL_PATH=C:\Keil_v5"
if not exist "%KEIL_PATH%\UV4\UV4.exe" (
    if exist "D:\application\KEIL\ARM\UV4\UV4.exe" set "KEIL_PATH=D:\application\KEIL\ARM"
)
set "UV4=%KEIL_PATH%\UV4\UV4.exe"
set "PROJECT_ROOT=%~dp0bootloader\project\MDK_V5"
set "UVPROJX=%PROJECT_ROOT%\bootloader.uvprojx"
set "OUTPUT_DIR=%PROJECT_ROOT%\objects"
set "LISTING_DIR=%PROJECT_ROOT%\listings"
set "LOG_FILE=%PROJECT_ROOT%\build_log.txt"

if /i "%~1"=="-Clean" goto :clean
if /i "%~1"=="clean" goto :clean
set "ACTION=-b"
set "ACTION_TEXT=Build"
if /i "%~1"=="-Rebuild" goto :rebuild
if /i "%~1"=="rebuild" goto :rebuild
if not "%~1"=="" (
    echo Usage: build.bat [rebuild ^| clean]
    exit /b 2
)
goto :dobuild

:rebuild
set "ACTION=-r"
set "ACTION_TEXT=Rebuild"

:dobuild
if not exist "%UV4%" (
    echo [ERROR] UV4.exe not found: %UV4%
    echo Set KEIL_PATH to your Keil installation directory.
    exit /b 2
)
if not exist "%UVPROJX%" (
    echo [ERROR] Project file not found: %UVPROJX%
    exit /b 2
)
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"
if not exist "%LISTING_DIR%" mkdir "%LISTING_DIR%"
echo ==== AT32F415 Bootloader %ACTION_TEXT% ====
echo Project: %UVPROJX%
echo Save source files before building.
pushd "%PROJECT_ROOT%"
if errorlevel 1 exit /b 2
start "" /wait "%UV4%" %ACTION% "%UVPROJX%" -t "bootloader" -j0 -o "%LOG_FILE%"
set "EXIT_CODE=%ERRORLEVEL%"
popd
if exist "%LOG_FILE%" type "%LOG_FILE%"
REM UV4 returns 1 for a successful build with warnings.
if not "%EXIT_CODE%"=="0" if not "%EXIT_CODE%"=="1" (
    echo [FAILED] Build failed. Exit code: %EXIT_CODE%
    exit /b %EXIT_CODE%
)

REM Always regenerate the raw BIN, including after an incremental build.
REM The project's AfterMake step also writes this BIN to listings.
set "FROMELF=%KEIL_PATH%\ARMCC\bin\fromelf.exe"
if not exist "%FROMELF%" set "FROMELF=%KEIL_PATH%\ARMCLANG\bin\fromelf.exe"
if not exist "%FROMELF%" set "FROMELF=%KEIL_PATH%\ARM\ARMCC\bin\fromelf.exe"
if not exist "%FROMELF%" set "FROMELF=%KEIL_PATH%\ARM\ARMCLANG\bin\fromelf.exe"
if not exist "%FROMELF%" (
    echo [ERROR] fromelf.exe not found under KEIL_PATH.
    exit /b 2
)
if not exist "%OUTPUT_DIR%\bootloader.axf" (
    echo [ERROR] Missing bootloader.axf. Do not use stale output.
    exit /b 2
)
"%FROMELF%" --bin --output "%LISTING_DIR%\bootloader.bin" "%OUTPUT_DIR%\bootloader.axf"
if errorlevel 1 (
    echo [FAILED] BIN export failed. Do not use stale output.
    exit /b 2
)
REM The factory packer reserves 13KB for the Bootloader.
for %%F in ("%LISTING_DIR%\bootloader.bin") do if %%~zF GTR 13312 (
    echo [ERROR] Bootloader exceeds the 13312-byte reserved area.
    exit /b 2
)
echo [SUCCESS] Bootloader build completed.
if "%EXIT_CODE%"=="1" echo [WARNING] Keil reported build warnings; see log above.
echo HEX: %OUTPUT_DIR%\bootloader.hex
echo AXF: %OUTPUT_DIR%\bootloader.axf
echo BIN: %LISTING_DIR%\bootloader.bin
echo Flash base: 0x08000000
echo For a factory image, merge this BIN with a separate APP BIN in the PC tool.
exit /b 0

:clean
echo ==== Clean Bootloader Build Output ====
REM Delete only generated files within the fixed project directories.
for %%E in (o d crf axf hex htm map lnp sct dep) do if exist "%OUTPUT_DIR%\*.%%E" del /q /f "%OUTPUT_DIR%\*.%%E"
if exist "%LISTING_DIR%\bootloader.bin" del /q /f "%LISTING_DIR%\bootloader.bin"
if exist "%LISTING_DIR%\*.lst" del /q /f "%LISTING_DIR%\*.lst"
if exist "%LISTING_DIR%\bootloader.map" del /q /f "%LISTING_DIR%\bootloader.map"
if exist "%LOG_FILE%" del /q /f "%LOG_FILE%"
echo [OK] Clean done
exit /b 0

