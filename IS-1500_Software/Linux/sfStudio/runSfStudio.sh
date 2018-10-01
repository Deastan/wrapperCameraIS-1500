#!/bin/sh
appname=sfStudio

dirname=`dirname $0`
tmp="${dirname#?}"

if [ "${dirname%$tmp}" != "/" ]; then
dirname=$PWD/$dirname
fi
LD_LIBRARY_PATH="$dirname/lib"
export LD_LIBRARY_PATH
"$dirname/$appname" "$@"
