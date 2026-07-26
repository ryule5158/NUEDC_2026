@echo off
rem Full release flow: rebuild the PS7 bitstream, package BOOT_PS7.bin,
rem program QSPI, and verify the written data.
call "%~dp0Build-QspiBoot.cmd" --no-pause
if errorlevel 1 goto :failed

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Program-Qspi.ps1"
if errorlevel 1 goto :failed

echo.
echo Full QSPI build, programming, and verification completed successfully.
pause
exit /b 0

:failed
echo.
echo Full QSPI release flow failed. Review the error above.
pause
exit /b 1

