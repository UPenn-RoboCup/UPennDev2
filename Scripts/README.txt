Place the upstart *.conf files into /etc/init/
Please the *.rules files in /etc/udev/rules.d/

For usb id: lsusb -v -s 001:007 | grep Serial
From hacakday
http://hackaday.com/2009/09/18/how-to-write-udev-rules/

udevadm info -a -p $(udevadm info -q path -n /dev/video2)

Udev anomaly
http://www.linuxquestions.org/questions/slackware-14/udev-rules-and-ttys0-device-644237/

Put rc.local into /etc/rc.local
