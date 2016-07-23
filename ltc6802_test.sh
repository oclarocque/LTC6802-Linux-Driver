#!/bin/sh

cd /sys/bus/iio/devices/iio:device0
scale=`cat scale`

echo "Cell #1:  discharge ON"
echo 1 > cell01_disch
sleep 0.1
echo "Cell #2:  discharge ON"
echo 1 > cell02_disch
sleep 0.1
echo "Cell #3:  discharge ON"
echo 1 > cell03_disch
sleep 0.1
echo "Cell #4:  discharge ON"
echo 1 > cell04_disch
sleep 0.1
echo "Cell #5:  discharge ON"
echo 1 > cell05_disch
sleep 0.1
echo "Cell #6:  discharge ON"
echo 1 > cell06_disch
sleep 0.1
echo "Cell #7:  discharge ON"
echo 1 > cell07_disch
sleep 0.1
echo "Cell #8:  discharge ON"
echo 1 > cell08_disch
sleep 0.1
echo "Cell #9:  discharge ON"
echo 1 > cell09_disch
sleep 0.1
echo "Cell #10: discharge ON"
echo 1 > cell10_disch
sleep 0.1
echo "Cell #11: discharge ON"
echo 1 > cell11_disch
sleep 0.1
echo "Cell #12: discharge ON"
echo 1 > cell12_disch
sleep 0.1
echo ""
echo "Cell #12: discharge OFF"
echo 0 > cell12_disch
sleep 0.1
echo "Cell #11: discharge OFF"
echo 0 > cell11_disch
sleep 0.1
echo "Cell #10: discharge OFF"
echo 0 > cell10_disch
sleep 0.1
echo "Cell #9:  discharge OFF"
echo 0 > cell09_disch
sleep 0.1
echo "Cell #8:  discharge OFF"
echo 0 > cell08_disch
sleep 0.1
echo "Cell #7:  discharge OFF"
echo 0 > cell07_disch
sleep 0.1
echo "Cell #6:  discharge OFF"
echo 0 > cell06_disch
sleep 0.1
echo "Cell #5:  discharge OFF"
echo 0 > cell05_disch
sleep 0.1
echo "Cell #4:  discharge OFF"
echo 0 > cell04_disch
sleep 0.1
echo "Cell #3:  discharge OFF"
echo 0 > cell03_disch
sleep 0.1
echo "Cell #2:  discharge OFF"
echo 0 > cell02_disch
sleep 0.1
echo "Cell #1:  discharge OFF"
echo 0 > cell01_disch
sleep 0.1
echo ""
adc=`cat in_voltage1_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #1:  $v mV"
sleep 0.1
adc=`cat in_voltage2-voltage1_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #2:  $v mV"
sleep 0.1
adc=`cat in_voltage3-voltage2_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #3:  $v mV"
sleep 0.1
adc=`cat in_voltage4-voltage3_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #4:  $v mV"
sleep 0.1
adc=`cat in_voltage5-voltage4_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #5:  $v mV"
sleep 0.1
adc=`cat in_voltage6-voltage5_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #6:  $v mV"
sleep 0.1
adc=`cat in_voltage7-voltage6_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #7:  $v mV"
sleep 0.1
adc=`cat in_voltage8-voltage7_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #8:  $v mV"
sleep 0.1
adc=`cat in_voltage9-voltage8_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #9:  $v mV"
sleep 0.1
adc=`cat in_voltage10-voltage9_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #10: $v mV"
sleep 0.1
adc=`cat in_voltage11-voltage10_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #11: $v mV"
sleep 0.1
adc=`cat in_voltage12-voltage11_raw`
v=`echo "scale=0; $adc * $scale /1" | bc`
echo "Cell #12: $v mV"
sleep 0.1