

set venv_dir=D:\Factory\pythonvenv

CALL %venv_dir%\Scripts\activate
chcp 65001

pyinstaller BurnMain.spec -F -w

deactivate