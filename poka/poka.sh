#!/bin/bash
#--------------------------------------------------------------------
#バックグラウンド実行用のスクリプト
#--------------------------------------------------------------------
#
#install:
# $ sudo ln -s /home/ubuntu/ros2_ws/src/poka/poka.service /etc/systemd/system
# $ sudo systemctl enable poka.service
# $ sudo systemctl start poka.service
# $ systemctl status poka.service
#--------------------------------------------------------------------

#変数の設定
SCRIPTDIR=/home/ubuntu/ros2_ws/src/poka/
LOGDIR=$SCRIPTDIR/log
ENVFILE=/home/ubuntu/ros2_ws/install/setup.bash

#遅延起動処理
#linuxが起動してからの秒数を取得
UPTIME_1=`/bin/cat /proc/uptime | /usr/bin/awk '{print $1}'`
#小数点を切り捨て
UPTIME_2=`echo "scale=0; ${UPTIME_1} / 1" | /usr/bin/bc`
UPTIME_3=$(( ${UPTIME_2} ))

# 200秒以上経っていなければsleep
#if [ ${UPTIME_3} -gt 200 ]; then
#  echo ""
#else
#  echo "boot waiting..."
#  sleep 10
#fi

#マルチキャストアドレスのルーティング先をローカルループバックアドレスに向ける
#LAN/WLANが未接続の時に、このコマンドを実行すること
#LAN_ENABLE_E=$((`cat /sys/class/net/eth0/carrier`))
#LAN_ENABLE_W=$((`cat /sys/class/net/wlan0/carrier`))
#if [ ]; then
#sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
#sudo ifconfig lo multicast
#fi

#実行
if [ -f ${ENVFILE} ]; then
    #環境変数読み込み
    echo "Loading ROS2 Env..."
    source /home/ubuntu/ros2_ws/install/setup.bash
    #ROS2 LAUNCH
    if [ -d ${LOGDIR} ]; then
        echo "ROS2 Launching..."
        #ネットワーク上で一意にするための値
        #export ROS_ALLOWED_HOSTS="localhost.local:robot_1.local"
        export ROS_DOMAIN_ID=99
        #roslaunch実行
        exec ros2 launch poka poka.launch.py >> ${LOGDIR}/poka.log 2>&1
    else
        #ログのディレクトリがないとき
        echo "There is no ${LOGDIR}"
    fi
else
    #環境変数ファイルがないとき
    echo "There is no ${ENVFILE}"
fi
