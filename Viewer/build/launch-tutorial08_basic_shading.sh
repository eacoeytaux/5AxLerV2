#!/bin/sh
bindir=$(pwd)
cd /Users/ethancoeytaux/Documents/Xcode/5AxLerV2/5AxLerV2/Viewer/
export 

if test "x$1" = "x--debugger"; then
	shift
	if test "x" = "xYES"; then
		echo "r  " > $bindir/gdbscript
		echo "bt" >> $bindir/gdbscript
		GDB_COMMAND-NOTFOUND -batch -command=$bindir/gdbscript  USERFILE_COMMAND-NOTFOUND 
	else
		"USERFILE_COMMAND-NOTFOUND"  
	fi
else
	"USERFILE_COMMAND-NOTFOUND"  
fi
