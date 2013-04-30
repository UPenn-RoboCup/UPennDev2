#!/bin/bash

PWD=`pwd`
SRCDIR=${PWD}/stage
DESDIR=/

echo $SRCDIR
echo $DESDIR

generate_link(){
  echo "$1 $2"
  local SRC=$1
  local DES=$2
  for file in $SRC/*
  do
    filename=${file##*/}
#    echo $filename
    if [[ ! -a $DES/$filename ]]; then
#      echo $SRC/$filename $DES/$filename
      ln -s $SRC/$filename $DES/$filename
      echo $DES/$filename >> install_files
    elif [[ -d $SRC/$filename ]]; then
      generate_link $SRC/$filename $DES/$filename
    fi
  done
}

remove_link(){
  while read line
  do
    echo -e "remove $line\n"
    rm $line
  done < install_files
  rm install_files
}

echo $1
case "$1" in
  'install')
    generate_link $SRCDIR $DESDIR;;
  'uninstall')
    remove_link;;
  *)
    echo "unknown command";;
esac

