@echo off
rem Program the existing BOOT_PS7.bin into QSPI and verify it.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Program-Qspi.ps1"
if errorlevel 1 (
  echo.
  echo QSPI programming failed. Review the error and the output log.
) else (
  echo.
  echo QSPI programming and verification completed successfully.
)
pause

