@echo off
REM Athena Firmware Flash Script
REM Flashes MPU, SPU, and TPU using their USB DFU serial numbers
REM Serial numbers from sn.txt:
REM   SPU (USB1): 206831725843
REM   MPU (USB2): 200364500000
REM   TPU (USB3): 200364500000

setlocal

REM Define paths
set PROGRAMMER="c:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set BUILD_DIR=build

REM USB port assignments for each MCU
set SPU_USB=usb1
set MPU_USB=usb1
set TPU_USB=usb1

REM Serial numbers for each MCU
set SPU_SN=206831725843
set MPU_SN=200364500000
set TPU_SN=200364500000

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
echo Connecting to MPU (SN: %MPU_SN%)...
%PROGRAMMER% -c port=%MPU_USB% -w "%ELF_FILE%" -v -s
if errorlevel 1 (
    echo ERROR: Failed to flash MPU
    exit /b 1
)
echo MPU flashed successfully!
if "%TARGET%"=="mpu" exit /b 0
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
echo Connecting to SPU (SN: %SPU_SN%)...
%PROGRAMMER% -c port=%SPU_USB% -w "%ELF_FILE%" -v -s
if errorlevel 1 (
    echo ERROR: Failed to flash SPU
    exit /b 1
)
echo SPU flashed successfully!
if "%TARGET%"=="spu" exit /b 0
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
echo Connecting to TPU (SN: %TPU_SN%)...
%PROGRAMMER% -c port=%TPU_USB% -w "%ELF_FILE%" -v -s
if errorlevel 1 (
    echo ERROR: Failed to flash TPU
    exit /b 1
)
echo TPU flashed successfully!
if "%TARGET%"=="tpu" exit /b 0
goto :eof

:flash_all
call :flash_spu
call :flash_mpu
call :flash_tpu
echo.
echo ========================================
echo All MCUs flashed successfully!
echo ========================================
exit /b 0