@echo off
REM echo %1

set BASE_PATH=%CD%

set SVN_VER=%1

set APP_HEX_NAME=%BASE_PATH%\..\..\Firmware\project\SES\Output\Release\Exe\Ring8.hex
set APP_ZIP_NAME=%BASE_PATH%\..\Ring%SVN_VER%.zip
set APP_VER=0x01

REM ..\nrfutil52.exe pkg generate --hw-version 52 --sd-req 0xB6 --application-version %APP_VER% --key-file ..\private.pem --application %APP_HEX_NAME% %APP_ZIP_NAME%
..\nrfutil.exe pkg generate --hw-version 52 --sd-req 0xB6 --application-version %APP_VER% --key-file ..\private.pem --application %APP_HEX_NAME% %APP_ZIP_NAME%

echo. 
echo %APP_HEX_NAME%
echo %APP_ZIP_NAME%

REM ..\nrfutil52.exe pkg display %APP_ZIP_NAME%
..\nrfutil.exe pkg display %APP_ZIP_NAME%
