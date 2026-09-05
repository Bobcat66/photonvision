@echo off
REM Usage: install_tbb.bat <version> [prefix]
setlocal
 
set "VERSION=%~1"
set "PREFIX=%~2"
if "%PREFIX%"=="" set "PREFIX=C:\oneTBB"
 
set "TAG=%VERSION%"
if /i not "%TAG:~0,1%"=="v" set "TAG=v%VERSION%"
 
set "DIR=%TEMP%\onetbb-%RANDOM%"
mkdir "%DIR%"
 
git clone --branch "%TAG%" --depth 1 https://github.com/uxlfoundation/oneTBB.git "%DIR%\src" || exit /b 1
cmake -S "%DIR%\src" -B "%DIR%\build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%PREFIX%" -DTBB_TEST=OFF || exit /b 1
cmake --build "%DIR%\build" --config Release --parallel %NUMBER_OF_PROCESSORS% || exit /b 1
cmake --install "%DIR%\build" --config Release || exit /b 1
 
rmdir /s /q "%DIR%"