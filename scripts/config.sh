#!/bin/bash
set -e

# $1 should be the hostname

if [ -z $1 ]; then
	echo "no host specified (first argument value)"
	exit
fi

cd $FINALLAB_HOME/java

if [ "$2" = "ant" ]; then
	ant clean; ant
fi
ssh $1 "mkdir -p $3finallab; mkdir -p $3finallab/data;"

scp -pr $FINALLAB_HOME/scripts/runproc_panda.sh $FINALLAB_HOME/config $FINALLAB_HOME/java/finallab.jar $FINALLAB_HOME/lcmtypes $1:$3finallab/

