#!/usr/bin/env bash
for x in $(ls cfg/*.sh)
do
	echo ${x::-3}
	bash $x > ${x::-3}
done
