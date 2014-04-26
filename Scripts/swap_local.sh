#!/bin/bash

# If currently on remote, then swap to local
file /usr/local | grep remote 1>/dev/null
if [ $? -eq 0 ]
then
  # Remove the current
  sudo rm /usr/local
	sudo ln -s /home/nao/local /usr/local
  echo "On local now!"
  exit
fi

# If currently on local, then swap to remote if available
mount -l -t fuse.sshfs | grep local 1>/dev/null
if [ $? -eq 0 ]
then
  sudo rm /usr/local
	sudo ln -s /home/nao/remote /usr/local
  echo "On remote now!"
  exit
fi

# Fallback
sudo rm /usr/local
sudo ln -s /home/nao/local /usr/local
