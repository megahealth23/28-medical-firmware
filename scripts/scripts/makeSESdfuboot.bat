
set BASE_PATH=%CD%

set SVN_VER=%1

set BL_HEX_NAME=%BASE_PATH%\..\..\BootLoader\secure_bootloader\pca10056_ble_debug\ses\Output\Release\Exe\secure_bootloader_ble_s140_pca10056_debug.hex
REM set BL_HEX_NAME=%BASE_PATH%\..\..\BootLoader\secure_bootloader\pca10056_ble_debug\arm5_no_packs\_build\SecBootloader_debug.hex

set BOOT_VER=0x31
set BOOT_ZIP_NAME=%BASE_PATH%\..\Ring-%BOOT_VER%-Boot.zip

..\nrfutil.exe pkg generate --hw-version 52 --sd-req 0xB6 --bootloader-version %BOOT_VER% --key-file ..\private.pem --bootloader %BL_HEX_NAME% %BOOT_ZIP_NAME%

..\nrfutil.exe pkg display %BOOT_ZIP_NAME%
