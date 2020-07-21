#!/bin/sh

# "daemonize" by deufalt. Use "--" to run in interactive mode
if [ "x$1" != "x--" ]; then
    $0 --  &>> /var/log/network_control.log &
	  exit 0
fi

ETH="eth0"
BRIDGE="br0"
WLAN="wlan0"
TIMEOUT_LINK=600
WLAN_CARD="0"
ISETHLINK="0"
CONF_FILE="/etc/routerManager.xml"
VPN_FILE="/etc/openvpn/client.conf"
RESOLV_FILE="/run/resolvconf/resolv.conf"
RESOLV_FILE_STATIC="/run/resolvconf/resolv.conf.static"
RESOLV_FILE_STANDALONE="/run/resolvconf/resolv.conf.standalone"
RESOLV_FILE_ETH_DHCP="/run/resolvconf/interface/"$ETH".dhclient"
RESOLV_FILE_WLAN_DHCP="/run/resolvconf/interface/"$WLAN".dhclient"
RESOLV_FILE_VPN="/run/resolvconf/resolv.conf.bak"
HOSTS_FILE="/etc/hosts"
WPA_CONF="/etc/wpa_supplicant.conf"
WPA_CONF_MANUAL="/etc/wpa_supplicant.conf.manual"
HOSTAPD_CONF="/etc/hostapd/hostapd.conf"
PID_NETWORK_CONTROL="/var/run/network_control.sh"
WATCHDOG_WLAN=`cat /proc/uptime |  awk -F'.' '{print $1}'`

if [ ! -f $CONF_FILE ]; then
    echo "Configuration file not found."
    exit 1;
fi

iwconfig $WLAN
if [ $? -eq 0 ] ; then
    WLAN_CARD="1"
fi

addInterfaceToBridge()
{
    brctl show $BRIDGE | grep $1 > /dev/null
    if [ $? -ne 0 ] ; then
        ifconfig $1 up
        brctl addif $BRIDGE $1
    fi
}

delInterfaceToBridge()
{
    brctl show $BRIDGE | grep $1 > /dev/null
    if [ $? -eq 0 ] ; then
        brctl delif $BRIDGE $1
    fi
}

addAllEthToBridge()
{
    ETH_INTERF=`ifconfig -a | grep ^eth | grep -v $ETH | cut -d' ' -f1`
    for e in $ETH_INTERF
    do
        addInterfaceToBridge $e
    done
}

setEthParameters()
{
    if [ "$ISETHEXTERNAL" = "1" ]; then
        delInterfaceToBridge $ETH
        if [ "$ISETHLINK" = "1" ] ; then
            if [ "$ISACCESSPOINT" = "0" ]; then
                releaseWlanParameters
            fi
            `route del default`
            if [ "$ISETHDHCP" = "1" ]; then
                echo "Set Dhcp"
                dhclient -e IF_METRIC=100 -pf /var/run/dhclient.$ETH.pid -lf /var/lib/dhcp/dhclient.$ETH.leases -1 $ETH &
                echo $! > /var/run/dhclient.$ETH.pid
            else
                echo "Set static IP"
                ifconfig $ETH $IPSTATIC netmask $NETMASKSTATIC
                `route add default gw $GATEWAYSTATIC`
            fi
        fi
    else
        addInterfaceToBridge $ETH
    fi
}

setWlanParameters()
{
    CONFIGURE="0"
    if [ "$ISETHEXTERNAL" = "0" ]; then
        CONFIGURE="1"
    fi
    if [ "$ISETHEEXTERNAL" = "1" ] && [ "$ISETHLINK" = "0" ]; then
        CONFIGURE="1"
    fi

    if [ "$ISACCESSPOINT" = "1" ]; then
        #Access Point
        cp $HOSTAPD_CONF".base" $HOSTAPD_CONF
        echo "ssid="$WLANSSID >> $HOSTAPD_CONF
        echo "channel="$WLANCHANNEL >> $HOSTAPD_CONF
        echo "wpa_passphrase="$WLANPASSWORD >> $HOSTAPD_CONF
        hostapd -P /var/run/hostapd.$WLAN.pid $HOSTAPD_CONF &
        echo $! > /var/run/hostapd.$WLAN.pid
        sleep 2
        addInterfaceToBridge $WLAN
    else
        if [ "$CONFIGURE" = "1" ]; then
            #Managed
            ifconfig $WLAN up
            delInterfaceToBridge $WLAN
            iwconfig $WLAN mode managed
            iwconfig $WLAN essid $WLANSSID
            echo "network={" > $WPA_CONF
            echo "ssid=\""$WLANSSID"\"" >> $WPA_CONF
            if [ "$WLANMODEKEY" = "none" ]; then
                echo "key_mgmt=NONE" >> $WPA_CONF
            fi
            if [ "$WLANMODEKEY" = "wep" ]; then
                echo "key_mgmt=NONE" >> $WPA_CONF
                echo "wep_key0=\""$WLANPASSWORD"\"" >> $WPA_CONF
            fi
            if [ "$WLANMODEKEY" = "wpa" ]; then
                echo "psk=\""$WLANPASSWORD"\"" >> $WPA_CONF
                echo "key_mgmt=WPA-PSK" >> $WPA_CONF
                echo "pairwise=CCMP TKIP" >> $WPA_CONF
                echo "group=CCMP TKIP" >> $WPA_CONF
            fi
            if [ "$WLANMODEKEY" = "manual" ]; then
                cat $WPA_CONF_MANUAL > $WPA_CONF
            else
                echo "}" >> $WPA_CONF
            fi
            wpa_supplicant -B -i$WLAN -c/etc/wpa_supplicant.conf -P/var/run/wpa_supplicant.$WLAN.pid
            `route del default`
            if [ "$ISWLANDHCP" = "1" ]; then
                echo "Set Dhcp"
                dhclient -e IF_METRIC=100 -pf /var/run/dhclient.$WLAN.pid -lf /var/lib/dhcp/dhclient.$WLAN.leases -1 $WLAN &
                echo $! > /var/run/dhclient.$WLAN.pid
            else
                echo "Set static IP"
                ifconfig $WLAN $IPSTATIC netmask $NETMASKSTATIC
                `route add default gw $GATEWAYSTATIC`
            fi
        fi
    fi
}

releaseEthParameters()
{
    if [ -f /var/run/dhclient.$ETH.pid ]; then
        echo "Release Dhcp"
        PID_DHCLIENT=`cat /var/run/dhclient.$ETH.pid`
        kill $PID_DHCLIENT
        rm /var/run/dhclient.$ETH.pid
    fi
    ifconfig $ETH 0.0.0.0 up
}

releaseWlanParameters()
{
    if [ -f /var/run/dhclient.$WLAN.pid ]; then
        echo "Release Dhcp"
        PID_DHCLIENT=`cat /var/run/dhclient.$WLAN.pid`
        kill $PID_DHCLIENT
        rm /var/run/dhclient.$WLAN.pid
    fi
    if [ -f /var/run/wpa_supplicant.$WLAN.pid ]; then
        PID_WPASUPPLICANT=`cat /var/run/wpa_supplicant.$WLAN.pid`
        kill $PID_WPASUPPLICANT
        rm /var/run/wpa_supplicant.$WLAN.pid
    fi
    if [ -f /var/run/hostapd.$WLAN.pid ]; then
        PID_HOSTAPD=`cat /var/run/hostapd.$WLAN.pid`
        kill $PID_HOSTAPD
        rm /var/run/hostapd.$WLAN.pid
    fi
    delInterfaceToBridge $WLAN
    ifconfig $WLAN 0.0.0.0 down
    sleep 1
    ifconfig $WLAN 0.0.0.0 up
}

createResolvFile()
{
    rm $RESOLV_FILE_STATIC
    if [ "$DNSDOMAIN" != "" ]; then
        echo "domain "$DNSDOMAIN >> $RESOLV_FILE_STATIC
    fi

    echo "nameserver "`gethostip -d control` >> $RESOLV_FILE_STATIC
    for s in $DNSSERVERS
    do
        echo "nameserver "$s >> $RESOLV_FILE_STATIC
    done

    if [ "$DNSSEARCH" != "" ]; then
        echo "search reem-lan "$DNSSEARCH" reem" >> $RESOLV_FILE_STATIC
    else
        echo "search reem-lan reem" >> $RESOLV_FILE_STATIC
    fi

    rm $RESOLV_FILE_STANDALONE
    echo "domain reem-lan" >> $RESOLV_FILE_STANDALONE
    echo "nameserver "`gethostip -d control` >> $RESOLV_FILE_STANDALONE
    echo "search reem-lan" >> $RESOLV_FILE_STANDALONE
}

loadFromFile()
{
    echo `grep "$1" $CONF_FILE | awk -F'>' '{print $2}' | awk -F'<' '{print $1}'`
}

reloadConfFile()
{
    ISETHEXTERNAL=$(loadFromFile "ethExternal")
    ISETHDHCP=$(loadFromFile "ethIsDhcp")

    IPSTATIC=$(loadFromFile "ip")
    NETMASKSTATIC=$(loadFromFile "netmask")
    GATEWAYSTATIC=$(loadFromFile "gateway")
    DNSSERVERS=$(loadFromFile "dnsServers")
    DNSDOMAIN=$(loadFromFile "dnsDomain")
    DNSSEARCH=$(loadFromFile "dnsSearch")

    ISACCESSPOINT=$(loadFromFile "wifiAccessPoint")
    ISWLANDHCP=$(loadFromFile "wifiIsDhcp")
    WLANSSID=$(loadFromFile "wifiSsid")
    WLANCHANNEL=$(loadFromFile "wifiChannel")
    WLANMODEKEY=$(loadFromFile "wifiModeKey")
    WLANPASSWORD=$(loadFromFile "wifiPassword")

    ISVPN=$(loadFromFile "vpnEnabled")
    NTPSERVER=$(loadFromFile "ntpServer")
    
    createResolvFile
}

testParameter()
{
    NEW=$(loadFromFile "$1") 
    if [ "$2" != "$NEW" ]; then
        echo 1
    else
        echo 0
    fi
}

changeOfConfParameters()
{
    CHANGEETH=0
    CHANGEWIFI=0
    if [ $(testParameter "ethExternal" $ISETHEXTERNAL) -ne 0 ]; then
        CHANGEETH=1 
        CHANGEWIFI=1
    fi
    if [ $(testParameter "ethIsDhcp" $ISETHDHCP) -ne 0 ]; then
        CHANGEETH=1
    fi
    if [ $(testParameter "ip" $IPSTATIC) -ne 0 ]; then
        CHANGEETH=1
        CHANGEWIFI=0
    fi
    if [ $(testParameter "netmask" $NETMASKSTATIC) -ne 0 ]; then
        CHANGEETH=1
        CHANGEWIFI=0
    fi
    if [ $(testParameter "gateway" $GATEWAYSTATIC) -ne 0 ]; then
        CHANGEETH=1
        CHANGEWIFI=0
    fi
    if [ $CHANGEETH -ne 0 ]; then
        restartEth
    fi

    if [ $(testParameter "wifiAccessPoint" $ISACCESSPOINT) -ne 0 ]; then
        CHANGEWIFI=1
    fi
    if [ $(testParameter "wifiIsDhcp" $ISWLANDHCP) -ne 0 ]; then
        CHANGEWIFI=1
    fi
    if [ $(testParameter "wifiSsid" $WLANSSID) -ne 0 ]; then
        CHANGEWIFI=1
    fi
    if [ $(testParameter "wifiChannel" $WLANCHANNEL) -ne 0 ]; then
        CHANGEWIFI=1
    fi
    if [ $(testParameter "wifiModeKey" $WLANMODEKEY) -ne 0 ]; then
        CHANGEWIFI=1
    fi
    if [ $(testParameter "wifiPassword" $WLANPASSWORD) -ne 0 ]; then
        CHANGEWIFI=1
    fi
    if [ $CHANGEWIFI -ne 0 ]; then
        restartWlan
    fi

    if [ $(testParameter "dnsServers" "$DNSSERVERS") -ne 0 ]; then
        DNSSERVERS=$(loadFromFile "dnsServers")
        createResolvFile
    fi
    if [ $(testParameter "dnsDomain" $DNSDOMAIN) -ne 0 ]; then
        DNSDOMAIN=$(loadFromFile "dnsDomain")
        createResolvFile
    fi
    if [ $(testParameter "dnsSearch" "$DNSSEARCH") -ne 0 ]; then
        DNSSEARCH=$(loadFromFile "dnsSearch")
        createResolvFile
    fi

    if [ $(testParameter "vpnEnabled" $ISVPN) -ne 0 ]; then
        ISVPN=$(loadFromFile "vpnEnabled")
    fi
    if [ $(testParameter "ntpServer" $NTPSERVER) -ne 0 ]; then
        NTPSERVER=$(loadFromFile "ntpServer")
    fi
}

testDnsStatus()
{
    FILE=""
    WIFI="0"
    if [ "$ISETHEXTERNAL" = "1" ]; then
        if [ "$ISETHLINK" = "1" ] ; then
            if [ "$ISETHDHCP" = "0" ]; then
                FILE=$RESOLV_FILE_STATIC
            else
                FILE=$RESOLV_FILE_ETH_DHCP
            fi
        else
            WIFI="1"
        fi
    else
        WIFI="1" 
    fi
    if [ "$WIFI" = "1" ]; then
        if [ "$ISACCESSPOINT" = "1" ]; then
            FILE=$RESOLV_FILE_STANDALONE
        else
            if [ "$ISWLANDHCP" = "0" ]; then
                FILE=$RESOLV_FILE_STATIC
            else
                FILE=$RESOLV_FILE_WLAN_DHCP
            fi
        fi
    fi
    if [ "$FILE" != "" ] && [ -f $FILE ]; then
        DIFF=`diff $FILE $RESOLV_FILE`
        if [ "$DIFF" != "" ]; then
            cp -f $FILE $RESOLV_FILE
        fi
    fi
}

testNtpStatus()
{
    if [ "$ISVPN" = "0" ]; then
        NAME_NTPSERVER="ntpserver"
        CURRENT_IP=`grep $NAME_NTPSERVER $HOSTS_FILE | awk -F ' ' '{print $1}'`
        if [ "$CURRENT_IP" != "$NTPSERVER" ];
        then
            sed -i 's!'$CURRENT_IP' * '$NAME_NTPSERVER'!'$NTPSERVER' '$NAME_NTPSERVER'!' $HOSTS_FILE
            service ntp restart
        fi
    fi
}

restartEth()
{
    reloadConfFile
    killall -9 openvpn
    sleep $ROUTEDELAY
    releaseEthParameters
    setEthParameters
    service openvpn restart
}

restartWlan()
{
    if [ "$WLAN_CARD" = "0" ]; then
        return
    fi
    reloadConfFile
    killall -9 openvpn
    sleep $ROUTEDELAY
    releaseWlanParameters
    setWlanParameters
    service openvpn restart
}

testWlanClientLink()
{
    NOW=`cat /proc/uptime |  awk -F'.' '{print $1}'`

    if test `iw dev $WLAN link | grep -q "Not connected"; echo $?` -eq 0; then
        TM_LINK=$(($WATCHDOG_WLAN + $TIMEOUT_LINK))
        if [ "$NOW" -gt "$TM_LINK" ]; then
            echo "Test Wlan Client timeout. Restart interface"
            restartWlan
            WATCHDOG_WLAN=$NOW
        fi
    else
        WATCHDOG_WLAN=$NOW
    fi   
}

testDhcpServerStatus()
{
    /etc/init.d/isc-dhcp-server status > /dev/null
    if [ $? -ne 0 ] ; then
        /etc/init.d/isc-dhcp-server restart
    fi
}

testLink()
{
    ethtool $ETH | grep "Link detected" | grep -q "yes"
    if [ $? -eq 0 ] ; then
        if [ "$ISETHLINK" = "0" ] ; then
            # there eth is link
            ISETHLINK="1"
            if [ "$ISETHEXTERNAL" = "1" ]; then
                if [ "$WLAN_CARD" = "1" ]; then
                    if [ "$ISACCESSPOINT" = "0" ]; then
                        releaseWlanParameters
                    fi
                fi
            fi
            restartEth
        fi
    else
        if [ "$ISETHLINK" = "1" ] ; then
            # no eth link
            ISETHLINK="0"
            releaseEthParameters
            if [ "$ISETHEXTERNAL" = "1" ]; then
                if [ "$WLAN_CARD" = "1" ]; then
                    if [ "$ISACCESSPOINT" = "0" ]; then
                        restartWlan
                    fi
                fi
            fi
        fi
    fi

    if [ "$WLAN_CARD" = "1" ]; then
        if [ "$ISACCESSPOINT" = "1" ]; then
            addInterfaceToBridge $WLAN
        else
            testWlanClientLink
        fi
    fi
}

ROUTEDELAY="1"
if [ -f $VPN_FILE ]; then
    ROUTEDELAY=`grep "route-delay" $VPN_FILE | awk '{print $2}'`
    ROUTEDELAY=`expr $ROUTEDELAY + 1`
    echo "Route Delay is "$ROUTEDELAY
    sed -i 's!^redirect-gateway!#redirect-gateway!' $VPN_FILE
    touch /etc/openvpn/no_default_gw
    touch /etc/openvpn/no_dns_gw
    echo "Disabled redirect-gateway"
fi

echo $$ > $PID_NETWORK_CONTROL
restartEth
restartWlan
while :
do
    changeOfConfParameters
    testDnsStatus
    testLink
    testNtpStatus
    testDhcpServerStatus
    addAllEthToBridge
    sleep 1
done
