# TODO: Choose the find command based on the REPO
REPO=RoboCup
# TODO: Add sshfs mount check


# This is for RoboCup old code
mkdir -p robot/RoboCup/Player/Lib
# Robot
find RoboCup/Lib/Platform/NaoV4 \( -regex '.*.so' -or -regex '.*.lua' \) -exec cp -v '{}' robot/RoboCup/Player/Lib/ \;
# Image
find RoboCup/Lib/Modules/ImageProc \( -regex '.*.so' -or -regex '.*.lua' \) -exec cp -v '{}' robot/RoboCup/Player/Lib/ \;
# Comm
find RoboCup/Lib/Modules/Comm \( -regex '.*.so' -or -regex '.*.lua' \) -exec cp -v '{}' robot/RoboCup/Player/Lib/ \;
# Util
find RoboCup/Lib/Modules/Util \( -regex '.*.so' -or -regex '.*.lua' \) -exec cp -v '{}' robot/RoboCup/Player/Lib/ \;


# This is for UPennDev new code
#mkdir -p robot/UPennDev/Lib
#find UPennDev/Modules \( -regex '.*.so' \) -exec cp -v '{}' robot/UPennDev/Lib/ \;
# This is for debug
#find $REPO \( -regex '.*.so' \) -exec file '{}' \;
