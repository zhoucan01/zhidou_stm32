@echo off
setlocal enabledelayedexpansion

if not defined KEIL_PATH set "KEIL_PATH=C:\Keil_v5"
if not exist "%KEIL_PATH%\UV4\UV4.exe" if exist "D:\application\KEIL\ARM\UV4\UV4.exe" set "KEIL_PATH=D:\application\KEIL\ARM"

REM ---- pyOCD / CMSIS-DAP configuration ----
REM Target name (lowercase) and Keil DFP pack providing the AT32F415 flash algorithm.
set "PYOCD_TARGET=at32f415rbt7"
set "PYOCD_PACK=%LOCALAPPDATA%\Arm\Packs\.Download\Keil.AT32F4xx_DFP.1.3.5.pack"
REM SWD clock frequency in Hz (lower it if you get SWD ACK errors).
set "PYOCD_FREQ=1000000"
REM Locate the pyOCD entry point (python -m pyocd).
set "PYOCD_PY="
for %%E in (
  "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
  "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
  "%LOCALAPPDATA%\Programs\Python\Python310\python.exe"
  "%ProgramFiles%\Python312\python.exe"
  "%ProgramFiles%\Python311\python.exe"
) do if not defined PYOCD_PY if exist %%E set "PYOCD_PY=%%~E"
if not defined PYOCD_PY for /f "delims=" %%I in ('where python.exe 2^>nul') do set "PYOCD_PY=%%I"

set "P=%~dp0"
set "M=%P%project\MDK_V5"
set "D=AT32F415RBT7"
set "LOG=%M%\flash_log.txt"

REM DBG: daplink(=pyocd/CMSIS-DAP, default) | jlink | stlink | msd(mass-storage copy)
REM ERASE: 1=erase whole chip before flashing, 0=skip erase (default, pyocd flash --erase=auto only erases sectors being written)
set "DBG=daplink" & set "SB=0" & set "BM=" & set "ERASE=0"
:pa
if "%~1"=="" goto pa2
if /i "%~1"=="daplink" set "DBG=daplink"
if /i "%~1"=="d" set "DBG=daplink"
if /i "%~1"=="jlink" set "DBG=jlink"
if /i "%~1"=="j" set "DBG=jlink"
if /i "%~1"=="stlink" set "DBG=stlink"
if /i "%~1"=="s" set "DBG=stlink"
if /i "%~1"=="msd" set "DBG=msd"
if /i "%~1"=="-SkipBuild" set "SB=1"
if /i "%~1"=="-s" set "SB=1"
if /i "%~1"=="-Rebuild" set "BM=-Rebuild"
if /i "%~1"=="-r" set "BM=-Rebuild"
if /i "%~1"=="-Erase" set "ERASE=1"
if /i "%~1"=="-e" set "ERASE=1"
shift & goto pa
:pa2

echo [%DBG%] Device=%D% Build=%BM% Skip=%SB% Erase=%ERASE%

if "%SB%"=="0" if exist "%P%build.bat" (
  call "%P%build.bat" %BM%
  if !ERRORLEVEL! GTR 1 exit /b 1
)

if /i "%DBG%"=="jlink" goto :do_jlink
if /i "%DBG%"=="stlink" goto :do_stlink
if /i "%DBG%"=="msd" goto :do_msd
goto :do_daplink

:do_jlink
set "JLINK_EXE="
if exist "%KEIL_PATH%\ARM\Segger\JLink.exe" set "JLINK_EXE=%KEIL_PATH%\ARM\Segger\JLink.exe"
if not defined JLINK_EXE if exist "%ProgramFiles%\SEGGER\J-Link\JLink.exe" set "JLINK_EXE=%ProgramFiles%\SEGGER\J-Link\JLink.exe"
if not exist "%JLINK_EXE%" (
  echo [ERROR] JLink.exe not found
  exit /b 1
)
set "JF=%TEMP%\j_%RANDOM%.jlink"
if "%ERASE%"=="1" (
  (echo device %D%&echo speed 4000&echo interface SWD&echo erase&echo loadfile "%M%\objects\code.hex"&echo r&echo g&echo qc)>!JF!
) else (
  (echo device %D%&echo speed 4000&echo interface SWD&echo loadfile "%M%\objects\code.hex"&echo r&echo g&echo qc)>!JF!
)
"%JLINK_EXE%" -AutoConnect 1 -CommanderScript "!JF!"
set "RC=!ERRORLEVEL!"
del !JF! 2>nul
goto :result

:do_stlink
if not exist "%KEIL_PATH%\UV4\UV4.exe" (
  echo [ERROR] UV4.exe not found
  exit /b 1
)
pushd "%M%"
"%KEIL_PATH%\UV4\UV4.exe" -f code.uvprojx -j0 -o "%LOG%"
set "RC=!ERRORLEVEL!"
popd
if exist "%LOG%" type "%LOG%"
goto :result

:do_daplink
REM ---- pyOCD: full-chip erase (optional) then flash via CMSIS-DAP ----
set "HEX_FILE=%M%\objects\code.hex"
set "BIN_FILE=%M%\objects\code.bin"
set "AXF_FILE=%M%\objects\code.axf"
set "FROMELF=%KEIL_PATH%\ARM\ARMCC\Bin\fromelf.exe"
if not exist "%FROMELF%" set "FROMELF=%KEIL_PATH%\ARM\ARMCC\bin\fromelf.exe"
REM Regenerate code.bin from code.axf so the binary is always in sync.
if exist "%AXF_FILE%" if exist "%FROMELF%" (
    "%FROMELF%" --bin --output "%BIN_FILE%" "%AXF_FILE%"
)

if not defined PYOCD_PY (
    echo [ERROR] python.exe not found. Please install Python 3.10+ and pyocd.
    echo         pip install pyocd
    exit /b 1
)

REM Prefer code.hex (pyOCD handles Intel HEX with correct base address).
set "FLASH_FILE=%HEX_FILE%"
if not exist "%FLASH_FILE%" set "FLASH_FILE=%BIN_FILE%"
if not exist "%FLASH_FILE%" (
    echo [ERROR] No code.hex or code.bin was generated for flashing.
    exit /b 1
)

if not exist "%PYOCD_PACK%" (
    echo [ERROR] Keil AT32F4xx DFP pack not found:
    echo         %PYOCD_PACK%
    echo         Install it via Keil Pack Installer or adjust PYOCD_PACK in flash.bat.
    exit /b 1
)

echo ---- Locating CMSIS-DAP probe ----
set "PROBE_LIST=%TEMP%\pyocd_probe_list.txt"
"%PYOCD_PY%" -m pyocd list -p > "%PROBE_LIST%" 2>&1
set "LIST_RC=!ERRORLEVEL!"
type "%PROBE_LIST%"
if !LIST_RC! NEQ 0 (
    echo [ERROR] pyocd list failed with rc=!LIST_RC!.
    del "%PROBE_LIST%" 2>nul
    exit /b 1
)
REM pyocd list exits 0 even with no probes connected - inspect the output text instead.
REM (for /f with a quoted command gets its outer quotes stripped by cmd /c, so use a temp file.)
set "PROBE_FOUND="
findstr /i /c:"CMSIS-DAP" /c:"DAPLink" "%PROBE_LIST%" >nul && set "PROBE_FOUND=1"
del "%PROBE_LIST%" 2>nul
if not defined PROBE_FOUND (
    echo [ERROR] No CMSIS-DAP debug probe connected. Plug in the probe and retry.
    echo         ^(pyocd would otherwise hang forever waiting for a probe.^)
    exit /b 1
)

if "%ERASE%"=="1" (
    echo ---- Erasing whole chip %PYOCD_TARGET% ----
    "%PYOCD_PY%" -m pyocd erase --chip --target %PYOCD_TARGET% --pack "%PYOCD_PACK%" --frequency %PYOCD_FREQ%
    set "RC=!ERRORLEVEL!"
    if !RC! NEQ 0 (
        echo [ERROR] Chip erase failed ^(rc=!RC!^). Check SWD wiring, target power, and clock.
        set "RC=2"
        goto :result
    )
    echo [OK] Chip erased.
)

echo ---- Flashing %FLASH_FILE% ----
REM -W: do not wait forever if the probe disappears mid-run
"%PYOCD_PY%" -m pyocd flash "%FLASH_FILE%" --target %PYOCD_TARGET% --pack "%PYOCD_PACK%" --frequency %PYOCD_FREQ% --erase=auto -W
set "RC=!ERRORLEVEL!"
if !RC! NEQ 0 (
    echo [ERROR] Flash failed ^(rc=!RC!^).
    set "RC=2"
    goto :result
)

echo ---- Reset and run ----
"%PYOCD_PY%" -m pyocd reset --target %PYOCD_TARGET% --pack "%PYOCD_PACK%" --frequency %PYOCD_FREQ% -W 2>nul
echo [OK] Firmware flashed and target reset.
set "RC=0"
goto :result

:do_msd
REM ---- Legacy DAPLink mass-storage drag-and-drop copy (no explicit erase) ----
set "BIN_FILE=%M%\objects\code.bin"
set "AXF_FILE=%M%\objects\code.axf"
set "HEX_FILE=%M%\objects\code.hex"
set "FROMELF=%KEIL_PATH%\ARM\ARMCC\Bin\fromelf.exe"
if not exist "%FROMELF%" set "FROMELF=%KEIL_PATH%\ARM\ARMCC\bin\fromelf.exe"
if exist "%AXF_FILE%" if exist "%FROMELF%" (
    "%FROMELF%" --bin --output "%BIN_FILE%" "%AXF_FILE%"
)

set "DAP_DRIVE="
for %%D in (C D E F G H I J K L M N O P Q R S T U V W X Y Z) do (
    if exist "%%D:\INFO_UF2.TXT" set "DAP_DRIVE=%%D:"
    if exist "%%D:\DETAILS.TXT" if "!DAP_DRIVE!"=="" set "DAP_DRIVE=%%D:"
)

if "%DAP_DRIVE%"=="" (
    echo [ERROR] DAPLink mass-storage drive not found.
    echo         Use 'flash.bat daplink' for pyOCD/CMSIS-DAP erase+flash instead.
    exit /b 1
)

if exist "%BIN_FILE%" (
    copy /Y "%BIN_FILE%" "%DAP_DRIVE%\" >nul
) else if exist "%HEX_FILE%" (
    copy /Y "%HEX_FILE%" "%DAP_DRIVE%\" >nul
) else (
    echo [ERROR] No code.bin or code.hex was generated for DAPLink download.
    exit /b 1
)

echo [OK] Firmware copied to %DAP_DRIVE%
set "RC=0"
goto :result

:result
if !RC! LEQ 1 (echo [OK]) else (echo [FAIL] !RC!)
exit /b !RC!
