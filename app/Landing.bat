@echo off
cd "%~dp0%"

SET PATH=C:/Program Files/OpenModelica1.25.4-64bit/bin/;C:/Program Files/OpenModelica1.25.4-64bit/lib//omc;C:/Program Files/OpenModelica1.25.4-64bit/lib/;C:/Users/iodyn/AppData/Roaming/.openmodelica/binaries/Platform;H:/SPACE/land/Platform/Resources/Library/ucrt64;H:/SPACE/land/Platform/Resources/Library/win64;H:/SPACE/land/Platform/Resources/Library;C:/Program Files/OpenModelica1.25.4-64bit/bin/;%PATH%;

SET ERRORLEVEL=

CALL .\Landing.exe -rt=1.0

SET RESULT=%ERRORLEVEL%

EXIT /b %RESULT%
