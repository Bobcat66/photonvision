@echo off
REM Usage: install_tbb.bat <version> [prefix]
REM Pulls the prebuilt Windows binary from the GitHub *release* for <version>
REM (not a git branch/tag clone) and extracts it into <prefix>.
setlocal

set "VERSION=%~1"
if /i "%VERSION:~0,1%"=="v" set "VERSION=%VERSION:~1%"
set "PREFIX=%~2"
if "%PREFIX%"=="" set "PREFIX=C:\tbb"
set "URL=https://github.com/uxlfoundation/oneTBB/releases/download/v%VERSION%/oneapi-tbb-%VERSION%-win.zip"

if not exist "%PREFIX%" mkdir "%PREFIX%"
curl -fsSL -o "%TEMP%\onetbb.zip" "%URL%" || exit /b 1
tar xf "%TEMP%\onetbb.zip" -C "%PREFIX%" --strip-components=1 || exit /b 1
del "%TEMP%\onetbb.zip"
