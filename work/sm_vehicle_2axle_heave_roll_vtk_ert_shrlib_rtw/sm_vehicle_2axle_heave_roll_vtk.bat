
call "setup_mingw.bat"

cd .

chcp 1252

if "%1"=="" ("%MINGW_ROOT%\mingw32-make.exe"  -f sm_vehicle_2axle_heave_roll_vtk.mk all) else ("%MINGW_ROOT%\mingw32-make.exe"  -f sm_vehicle_2axle_heave_roll_vtk.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1