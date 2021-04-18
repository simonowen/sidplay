@echo off
setlocal
set NAME=sidplay

if "%1"=="clean" goto clean

pasmo -v --tapbas %NAME%.asm %NAME%.tap %NAME%.sym
if errorlevel 1 goto end

if "%1"=="run" start %NAME%.tap
goto end

:clean
del /q %NAME%.tap %NAME%.sym %NAME%.o 2>nul

:end
endlocal
