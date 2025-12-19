@echo off
set count=1

:wait
    "%~dp0\dfu-util.exe" -l -d 0483:df11 | findstr "Found" >NUL 2>&1
    if %errorlevel% == 0 (
        "%~dp0\STM32Programmer\bin\STM32_Programmer_CLI.exe" -c port=usb1 -d %3 -v
    ) else (
        if %count% gtr 1000 goto break 
        echo %count%
        set /A count+=1
        ping -n 1 127.0.0.1 >NUL
        goto :wait
    )

:finish
