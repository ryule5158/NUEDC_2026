@echo off
setlocal
rem Safe public programming entry:
rem always clean-rebuild the current AD9238 project before touching QSPI.
call "%~dp0Build-And-Program-Qspi.cmd"
set "PROGRAM_RESULT=%ERRORLEVEL%"
endlocal & exit /b %PROGRAM_RESULT%
