@echo off
setlocal
rem Program the newest G-analyzer BOOT image into QSPI and verify it.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Program-Qspi.ps1" -Image "%~dp0..\output\BOOT_G_ANALYZER_AD9238.bin"
set "PROGRAM_RESULT=%ERRORLEVEL%"
if not "%PROGRAM_RESULT%"=="0" goto :failed

echo.
echo QSPI programming and verification completed successfully.
pause
endlocal
exit /b 0

:failed
echo.
echo QSPI programming failed. Review the error and output log.
pause
endlocal & exit /b %PROGRAM_RESULT%
