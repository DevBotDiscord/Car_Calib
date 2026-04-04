@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "BASH_EXE="

if defined GIT_BASH_EXE if exist "%GIT_BASH_EXE%" set "BASH_EXE=%GIT_BASH_EXE%"
if not defined BASH_EXE if exist "%ProgramFiles%\Git\bin\bash.exe" set "BASH_EXE=%ProgramFiles%\Git\bin\bash.exe"
if not defined BASH_EXE if exist "%ProgramFiles%\Git\usr\bin\bash.exe" set "BASH_EXE=%ProgramFiles%\Git\usr\bin\bash.exe"
if not defined BASH_EXE if exist "%ProgramW6432%\Git\bin\bash.exe" set "BASH_EXE=%ProgramW6432%\Git\bin\bash.exe"
if not defined BASH_EXE if exist "%ProgramW6432%\Git\usr\bin\bash.exe" set "BASH_EXE=%ProgramW6432%\Git\usr\bin\bash.exe"

if not defined BASH_EXE (
    for %%I in (bash.exe) do set "BASH_EXE=%%~$PATH:I"
)

if not defined BASH_EXE (
    echo [ERROR] bash.exe was not found.
    echo [ERROR] Install Git for Windows or set GIT_BASH_EXE to your bash.exe path.
    exit /b 1
)

"%BASH_EXE%" "%SCRIPT_DIR%deploy_production.sh" %*
set "EXIT_CODE=%ERRORLEVEL%"

endlocal & exit /b %EXIT_CODE%
