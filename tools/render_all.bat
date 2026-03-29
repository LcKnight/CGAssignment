@echo off
setlocal

set ROOT=%~dp0
set MINGW=C:\Compilers\msys64\mingw64\bin
set PATH=%MINGW%;%PATH%
set EXE=%ROOT%build\windows-mingw-release\PathTracer.exe

if not exist "%EXE%" (
    echo ERROR: PathTracer.exe not found at %EXE%
    pause
    exit /b 1
)

echo === Rendering cornell-box (64 spp) ===
"%EXE%" "%ROOT%example-scenes-cg25\cornell-box" 64

echo === Rendering veach-mis (64 spp) ===
"%EXE%" "%ROOT%example-scenes-cg25\veach-mis" 64

echo === Rendering living-room (64 spp) ===
"%EXE%" "%ROOT%example-scenes-cg25\living-room" 64

echo.
echo All renders complete. Check cornell-box.png, veach-mis.png, living-room.png
pause
