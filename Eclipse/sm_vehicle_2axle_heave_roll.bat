
rem call "setup_mingw.bat"
set "MINGW_ROOT=C:\msys64\mingw64\bin"
@set "PATH=%PATH%;%MINGW_ROOT%"

cd .
echo Current path %cd%

chcp 1252

if "%1"=="" ("%MINGW_ROOT%\mingw32-make.exe"  -f sm_vehicle_2axle_heave_roll.mk all) else ("%MINGW_ROOT%\mingw32-make.exe"  -f sm_vehicle_2axle_heave_roll.mk %1)

pause on

@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1