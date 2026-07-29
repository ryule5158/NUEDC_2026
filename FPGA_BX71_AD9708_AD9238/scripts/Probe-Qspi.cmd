@echo off
rem Read-only check: locate tools, connect JTAG, and validate one XC7Z020.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Program-Qspi.ps1" -ProbeOnly
if errorlevel 1 (
  echo.
  echo QSPI programmer probe failed. Review the error above.
) else (
  echo.
  echo QSPI programmer probe completed successfully. Flash was not changed.
)
pause

