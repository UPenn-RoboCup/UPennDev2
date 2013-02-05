#!/bin/bash -e

ARCH=`uname -m`

case "$ARCH" in
  'i686')
    MODE=i386;;
  'x86_64')
    MODE=x86-64;;
  *)
    MODe=i386;;
esac

WEBOTS644_SRC="http://www.cyberbotics.com/archive/linux/webots-6.4.4-$MODE.tar.bz2"
WEBOTS702_SRC="http://www.cyberbotics.com/dvd/linux/webots/webots-7.0.2-$MODE.tar.bz2"
WEBOTS644_FILE="webots-6.4.4-$MODE.tar.bz2"
WEBOTS702_FILE="webots-7.0.2-$MODE.tar.bz2"
WEBOTS644_DIR="webots-6.4.4"
WEBOTS702_DIR="webots-7.0.2"
USB_DONGLE_DRIVER_SRC="http://www.cyberbotics.com/dvd/linux/webots/driver_usb_dongle/matrix-rule-handler"
USB_DONGLE_DRIVER_FILE="matrix-rule-handler"



echo "Install webots 6.4.4"
echo $WEBOTS644_SRC
if [ ! -f $WEBOTS644_FILE.done ]; then
  wget $WEBOTS644_SRC
  touch $WEBOTS644_FILE.done
fi

echo $WEBOTS702_SRC
if [ ! -f $WEBOTS702_FILE.done ]; then
  wget $WEBOTS702_SRC
  touch $WEBOTS702_FILE.done
fi

if [ ! -f $WEBOTS644_DIR.done ]; then
  mkdir -p `pwd`/$WEBOTS644_DIR
  tar -xvf $WEBOTS644_FILE -C $WEBOTS644_DIR
fi

if [ ! -f $WEBOTS702_DIR.done ]; then
  mkdir -p `pwd`/$WEBOTS702_DIR
  tar -xvf $WEBOTS702_FILE -C $WEBOTS702_DIR
fi

mv $WEBOTS644_DIR /usr/local/.
rm -rf $WEBOTS644_DIR.done

mv $WEBOTS702_DIR /usr/local/.
rm -rf $WEBOTS702_DIR.done

ln -s /usr/local/webots-6.4.4/webots /usr/local/webots
ln -s /usr/local/webots-6.4.4/webots/webots /usr/local/bin/webots

if [ ! -f $USB_DONGLE_DRIVER_FILE.done ]; then
  wget $USB_DONGLE_DRIVER_SRC
  touch $USB_DONGLE_DRIVER_FILE.done
fi

chmod a+x $USB_DONGLE_DRIVER_FILE
chown root:root $USB_DONGLE_DRIVER_FILE
if [ ! -d /etc/udev/scripts ]; then
  mkdir /etc/udev/scripts
fi
mv $USB_DONGLE_DRIVER_FILE /etc/udev/scripts
rm $USB_DONGLE_DRIVER_FILE.done
echo "SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="0e50", ATTR{idProduct}=="000[1-9]", PROGRAM="/etc/udev/scripts/matrix-rule-handler"" >> /etc/udev/rules.d/99-matrix.rules
echo "SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="0e50", ATTR{idProduct}=="000[1-9]", MODE="0666"" >> /etc/udev/rules.d/99-matrix.rules
chmod 644 /etc/udev/rules.d/99-matrix.rules

rm -rf $WEBOTS644_FILE.done
rm -rf $WEBOTS644_FILE
rm -rf $WEBOTS702_FILE.done
rm -rf $WEBOTS702_FILE
