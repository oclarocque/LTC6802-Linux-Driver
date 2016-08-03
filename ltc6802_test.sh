#!/bin/sh

cd /sys/bus/iio/devices/iio:device0
scale=$(cat scale)

for i in $(seq -w 1 12);
do
	echo "Cell #${i}: discharge ON"
	echo 1 > "cell${i}_disch"
	ret=$(cat "cell${i}_disch")
	if [ "$ret" = "1" ]; then
		echo "OK"
	else
		echo "FAIL"
	fi
	#sleep 0.1
done

for i in $(seq -w 1 12);
do
	echo "Cell #${i}: discharge ON"
	echo 0 > "cell${i}_disch"
	ret=$(cat "cell${i}_disch")
	if [ "$ret" = "0" ]; then
		echo "OK"
	else
		echo "FAIL"
	fi
	#sleep 0.1
done

for i in $(seq 1 12);
do
	if [ "$i" -eq 1 ]; then
		adc=$(cat in_voltage${i}_raw)
	else
		j=$(($i - 1))
		adc=$(cat in_voltage${i}-voltage${j}_raw)
	fi
	#v=$(echo "scale=0; $adc * $scale /1" | bc)
	#echo "Cell #${i}:\t$v mV"
	echo -e "Cell #${i}:\t$adc"
	#sleep 0.1
done
