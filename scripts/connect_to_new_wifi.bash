#!/bin/bash

# Use as ./connect_to_new_wifi.bash <wifi_ssid> <pass> <connection_priority>

sudo sh -c 'nmcli d wifi connect $0 password $1 && \
            nmcli connection modify hackaton_d connection.permissions "" && \
            nmcli connection modify hackaton_d connection.autoconnect-priority $2' $1 $2 $3
