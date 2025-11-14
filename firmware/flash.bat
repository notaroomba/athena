@echo off
REM Athena Firmware Flash Script
REM Flashes MPU, SPU, and TPU using their USB port assignments
REM Port assignments:
REM   SPU: USB1
REM   MPU: USB2
REM   TPU: USB3

setlocal

REM Configuration - Set to 1 to run firmware after flashing, 0 to halt
set RUN_AFTER_FLASH=1

REM Define paths
set PROGRAMMER="c:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set BUILD_DIR=build

REM USB port assignments for each MCU (edit these if your connections differ)
set SPU_USB=usb1
set MPU_USB=usb2
set TPU_USB=usb3

REM Set run flag based on configuration
if "%RUN_AFTER_FLASH%"=="1" (
    set RUN_FLAG=-s
) else (
    set RUN_FLAG=
)

REM Check if programmer exists
if not exist %PROGRAMMER% (
    echo ERROR: STM32_Programmer_CLI.exe not found!
    echo Please install STM32CubeProgrammer or update the path in this script.
    exit /b 1
)

REM Check if build directory exists
if not exist "%BUILD_DIR%" (
    echo ERROR: Build directory not found!
    echo Please run 'make debug' or 'make release' first.
    exit /b 1
)

REM Parse command line arguments
set TARGET=%1
set CONFIG=%2

if "%TARGET%"=="" (
    echo Usage: flash.bat [TARGET] [CONFIG]
    echo.
    echo Targets:
    echo   mpu       - Flash MPU only
    echo   spu       - Flash SPU only
    echo   tpu       - Flash TPU only
    echo   all       - Flash all three MCUs
    echo.
    echo Configs:
    echo   debug     - Flash debug build
    echo   release   - Flash release build
    echo.
    echo Examples:
    echo   flash.bat mpu debug
    echo   flash.bat all release
    echo   flash.bat spu debug
    exit /b 0
)

if "%CONFIG%"=="" (
    echo ERROR: Please specify configuration: debug or release
    exit /b 1
)

REM Validate config
if not "%CONFIG%"=="debug" if not "%CONFIG%"=="release" (
    echo ERROR: Invalid configuration. Use 'debug' or 'release'
    exit /b 1
)

REM Flash functions
if "%TARGET%"=="mpu" goto flash_mpu
if "%TARGET%"=="spu" goto flash_spu
if "%TARGET%"=="tpu" goto flash_tpu
if "%TARGET%"=="all" goto flash_all

echo ERROR: Invalid target. Use 'mpu', 'spu', 'tpu', or 'all'
exit /b 1

:flash_mpu
echo.
echo ========================================
echo Flashing MPU (%CONFIG%)
echo ========================================
set ELF_FILE=%BUILD_DIR%\MPU-%CONFIG%.elf
if not exist "%ELF_FILE%" (
    echo ERROR: %ELF_FILE% not found!
    exit /b 1
)
echo Connecting to MPU on %MPU_USB%...
%PROGRAMMER% -c port=%MPU_USB% -w "%ELF_FILE%" -v %RUN_FLAG%
if errorlevel 1 (
    echo ERROR: Failed to flash MPU
    exit /b 1
)
echo MPU flashed successfully!
if "%TARGET%"=="mpu" exit /b 0
REM After MPU is flashed and disconnects, TPU moves from usb3 to usb2
set TPU_USB=usb2
goto :eof

:flash_spu
echo.
echo ========================================
echo Flashing SPU (%CONFIG%)
echo ========================================
set ELF_FILE=%BUILD_DIR%\SPU-%CONFIG%.elf
if not exist "%ELF_FILE%" (
    echo ERROR: %ELF_FILE% not found!
    exit /b 1
)
echo Connecting to SPU on %SPU_USB%...
%PROGRAMMER% -c port=%SPU_USB% -w "%ELF_FILE%" -v %RUN_FLAG%
if errorlevel 1 (
    echo ERROR: Failed to flash SPU
    exit /b 1
)
echo SPU flashed successfully!
if "%TARGET%"=="spu" exit /b 0
REM After SPU is flashed and disconnects, other devices move up
REM MPU moves from usb2 to usb1, TPU moves from usb3 to usb2
set MPU_USB=usb1
set TPU_USB=usb2
goto :eof

:flash_tpu
echo.
echo ========================================
echo Flashing TPU (%CONFIG%)
echo ========================================
set ELF_FILE=%BUILD_DIR%\TPU-%CONFIG%.elf
if not exist "%ELF_FILE%" (
    echo ERROR: %ELF_FILE% not found!
    exit /b 1
)
echo Connecting to TPU on %TPU_USB%...
%PROGRAMMER% -c port=%TPU_USB% -w "%ELF_FILE%" -v %RUN_FLAG%
if errorlevel 1 (
    echo ERROR: Failed to flash TPU
    exit /b 1
)
echo TPU flashed successfully!
if "%TARGET%"=="tpu" exit /b 0
goto :eof

:flash_all
call :flash_tpu
call :flash_mpu
call :flash_spu
echo.
echo ========================================
echo All MCUs flashed successfully!
echo ========================================
exit /b 0