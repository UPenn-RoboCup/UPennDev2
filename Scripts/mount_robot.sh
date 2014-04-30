ROBOT=$1
getent hosts | grep $ROBOT

if [ $? -eq 0 ]
then
sshfs $ROBOT:local remote
sshfs $ROBOT: robot
fi
