@echo off

if "%1"=="clean" goto clean

pyz80.py --exportfile=sidplay.sym sidplay.asm
goto end

:clean
if exist sidplay.dsk del sidplay.dsk sidplay.sym

:end
