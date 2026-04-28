@echo off
REM echo %1

set BASE_PATH=%CD%

set APP_HEX_NAME=%BASE_PATH%\..\..\Firmware\project\MDK\_build\Ring8.hex
set BTS_HEX_NAME=%BASE_PATH%\..\bootloader_settings_release.hex

REM ..\nrfutil52.exe settings generate --family NRF52840 --application %APP_HEX_NAME% --application-version 1 --bootloader-version 1 --bl-settings-version 2 %BTS_HEX_NAME%

..\nrfutil.exe settings generate --family NRF52840 --application %APP_HEX_NAME% --application-version 1 --bootloader-version 1 --bl-settings-version 2 %BTS_HEX_NAME%