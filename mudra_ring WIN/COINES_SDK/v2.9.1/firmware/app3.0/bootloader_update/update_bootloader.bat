:: When bootloader update is complete, the board is switched to Application mode


@echo off
..\..\..\tools\app_switch\app_switch usb_dfu_bl
..\..\..\tools\usb-dfu\dfu-util --device -,108c:ab3d -a RAM -D usb_ble_dfu_bootloader.pkg -R
pause
 