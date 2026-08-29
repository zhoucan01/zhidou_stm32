@echo off
setlocal enabledelayedexpansion

if not defined KEIL_PATH set "KEIL_PATH=C:\Keil_v5"
if not exist "%KEIL_PATH%\UV4\UV4.exe" if exist "D:\application\KEIL\ARM\UV4\UV4.exe" set "KEIL_PATH=D:\application\KEIL\ARM"
set "P=%~dp0" & set "M=%P%MDK-ARM" & set "D=STM32F103C8" & set "LOG=%P%MDK-ARM\code\flash_log.txt"

set "DBG=stlink" & set "SB=0" & set "BM="
:pa
if "%~1"=="" goto pa2
if /i "%~1"=="jlink" set "DBG=jlink"
if /i "%~1"=="j" set "DBG=jlink"
if /i "%~1"=="stlink" set "DBG=stlink"
if /i "%~1"=="s" set "DBG=stlink"
if /i "%~1"=="-SkipBuild" set "SB=1"
if /i "%~1"=="-s" set "SB=1"
if /i "%~1"=="-Rebuild" set "BM=-Rebuild"
if /i "%~1"=="-r" set "BM=-Rebuild"
shift & goto pa
:pa2

echo [%DBG%] Build=%BM% Skip=%SB%

if "%SB%"=="0" if exist "%P%build.bat" (
  call "%P%build.bat" %BM%
  if !ERRORLEVEL! GTR 1 exit /b 1
)

if /i "%DBG%"=="jlink" goto :do_jlink
goto :do_stlink

:do_jlink
if not exist "%KEIL_PATH%\ARM\Segger\JLink.exe" (
  echo [ERROR] JLink.exe not found
  exit /b 1
)
set "JF=%TEMP%\j_%RANDOM%.jlink"
(echo device %D%&echo speed 4000&echo interface SWD&echo loadfile "%M%\code\code.hex"&echo r&echo g&echo qc)>!JF!
"%KEIL_PATH%\ARM\Segger\JLink.exe" -AutoConnect 1 -CommanderScript "!JF!"
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

:result
if !RC! LEQ 1 (echo [OK]) else (echo [FAIL] !RC!)
exit /b !RC!
