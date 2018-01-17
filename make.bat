@echo off
setlocal
set NAME=sidplay

if "%1"=="clean" goto clean

pyz80.py -I samdos2 --mapfile=%NAME%.map %NAME%.asm
if errorlevel 1 goto end

if "%1"=="run" start %NAME%.dsk
if "%1"=="net" SAMdisk %NAME%.dsk sam:
goto end

:clean
del /q %NAME%.dsk %NAME%.map 2>nul

:end
endlocal
