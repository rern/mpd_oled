#!/bin/bash

. /srv/http/bash/addons.sh

installstart "$1"

getinstallzip

chmod +x /usr/bin/mpd_oled*

getuninstall

if [[ ${args[0]} == i2c ]]; then
	echo dtparam=i2c_arm=on >> /boot/config.txt
	echo i2c-dev >> /etc/modules-load.d/raspberrypi.conf
else
	echo dtparam=spi=on >> /boot/config.txt
fi

systemctl daemon-reload
systemctl enable --now mpd_oled

installfinish

echo -e "$info Reboot required."
