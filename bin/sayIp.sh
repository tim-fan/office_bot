#!/bin/bash

#say the wlan0 ip out loud forever

while true
do
    
    ip="$(ifconfig | grep -A 1 'wlan0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
    ip_words=`echo $ip | sed -e 's/\./ dot /g' -e 's/[0123456789]/ & /g'`
    
    if [ ! -z "$ip" ] 
    then
        espeak -s 100 "My address is ${ip_words}"
    else
        espeak -s 100 "I have no address"
    fi
    sleep 3
done
