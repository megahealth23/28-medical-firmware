@echo off
REM echo %1

set BASE_PATH=%CD%

set BOOT_VER=0x30

set APP_VER=%2

set SD_HEX_NAME=%BASE_PATH%\..\s140_nrf52_6.1.1_softdevice.hex

REM set BL_HEX_NAME=%BASE_PATH%\..\..\BootLoader\secure_bootloader\pca10056_ble\arm5_no_packs\_build\SecBootloader.hex
set BL_HEX_NAME=%BASE_PATH%\..\..\BootLoader\secure_bootloader\pca10056_ble_debug\ses\Output\Release\Exe\secure_bootloader_ble_s140_pca10056_debug.hex
REM set BL_HEX_NAME=%BASE_PATH%\..\..\BootLoader\secure_bootloader\pca10056_ble_debug\arm5_no_packs\_build\SecBootloader_debug.hex

set APP_HEX_NAME=%BASE_PATH%\..\..\Firmware\project\SES\Output\Release\Exe\Ring8.hex
set MERGE_HEX_NAME=%BASE_PATH%\..\Bootloader_Rin8_V%APP_VER%_full_release.hex

set BLSETTINGS_HEX_NAME=%BASE_PATH%\..\bootloader_settings_release.hex
echo %SD_HEX_NAME%
echo %BLSETTINGS_HEX_NAME%
echo %BL_HEX_NAME%
echo %APP_HEX_NAME%

REM ..\nrfutil52.exe settings generate --family NRF52840 --application %APP_HEX_NAME% --application-version 1 --bootloader-version 1 --bl-settings-version 1 %BLSETTINGS_HEX_NAME%
..\nrfutil.exe settings generate --family NRF52840 --application %APP_HEX_NAME% --application-version 1 --bootloader-version %BOOT_VER% --bl-settings-version 2 %BLSETTINGS_HEX_NAME%

mergehex --merge %BL_HEX_NAME% %SD_HEX_NAME% --output production_final1.hex
mergehex --merge production_final1.hex %APP_HEX_NAME% --output production_final2.hex
mergehex --merge production_final2.hex %BLSETTINGS_HEX_NAME% --output %MERGE_HEX_NAME%


del production_final1.hex
del production_final2.hex
del %BLSETTINGS_HEX_NAME%