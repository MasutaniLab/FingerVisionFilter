#!/bin/bash
fullpath="$(realpath "$0")"
dir=${fullpath%/*}
filename=${fullpath##*/}
name=${filename%.*}
cd $dir
gnome-terminal --command="bash -c \"echo -ne '\033]0;${name} $*\007'; python ./${name}.py $*\""
