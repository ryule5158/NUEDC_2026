@echo off
setlocal
rem One-click persistent release flow:
rem 1. Clean rebuild from the current G-analyzer sources.
rem 2. Reject timing/DRC/stale-artifact failures.
rem 3. Package FSBL + fresh bitstream.
rem 4. Require hardware JTAG boot mode.
rem 5. Program QSPI at offset 0 and readback-verify it.

call "%~dp0Build-QspiBoot.cmd" --no-pause
set "BUILD_RESULT=%ERRORLEVEL%"
if not "%BUILD_RESULT%"=="0" goto :build_failed

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Program-Qspi.ps1" -Image "%~dp0..\output\BOOT_G_ANALYZER.bin"
set "PROGRAM_RESULT=%ERRORLEVEL%"
if not "%PROGRAM_RESULT%"=="0" goto :program_failed

echo.
echo Fresh G-analyzer project was built, programmed to QSPI, and verified.
echo Power off, select QSPI boot mode, then power on for the cold-boot test.
pause
endlocal
exit /b 0

:build_failed
echo.
echo Build/package failed. QSPI was not changed.
pause
endlocal & exit /b %BUILD_RESULT%

:program_failed
echo.
echo QSPI programming did not complete. Review the exact error above.
echo If boot mode is not JTAG: power off, select JTAG, power on, then rerun.
pause
endlocal & exit /b %PROGRAM_RESULT%
