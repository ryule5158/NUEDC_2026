@echo off
setlocal
rem Build a fresh timing-clean G-analyzer bitstream and BOOT image.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Build-QspiBoot.ps1"
set "BUILD_RESULT=%ERRORLEVEL%"
if not "%BUILD_RESULT%"=="0" goto :failed

echo.
echo Fresh G-analyzer bitstream and BOOT image generation completed.
if /I not "%~1"=="--no-pause" pause
endlocal
exit /b 0

:failed
echo.
echo Build failed. No stale BOOT image will be programmed.
if /I not "%~1"=="--no-pause" pause
endlocal & exit /b %BUILD_RESULT%
