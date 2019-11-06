#!/bin/bash
#--------------------------------------------------------------------
#ROSの信号監視
#--------------------------------------------------------------------
#
#install:
# $ sudo ln -s /home/pi/[INSTALL DIR]/roswatcher.service /etc/systemd/system
# $ sudo systemctl enable roswatcher.service
# $ sudo systemctl start roswatcher.service
#--------------------------------------------------------------------
#ref:
#
# ※必ずbashを使うこと。shは関数の宣言でエラーが出ます
#   https://yatta47.hateblo.jp/entry/2014/04/20/222707
# ※test式
#   http://itdoc.hitachi.co.jp/manuals/3020/30203S3530/JPAS0134.HTM
#

#----------------------------------
#定数
#----------------------------------

#異常ランプのBCM番号
SET_LAMP_PORT=6

#監視ループの間隔(sec)
SET_WATCH_INTERVAL=20

#----------------------------------
#メイン処理
#----------------------------------

#実行中ユーザをチェック
#if [ `/usr/bin/id -u` != "0" ]; then
#    #rootでなければ異常終了
#    echo " ! Please run as root."
#    exit 1
#fi

#主要コマンドの存在をチェック
if [ ! -x /usr/local/bin/gpio ]; then
    #見つからなければ異常終了
    echo " ! No 'gpio' Command."
    exit 1
fi
if [ ! -x /bin/ps ]; then
    echo " ! No 'ps' Command."
    exit 1
fi
if [ ! -x /bin/grep ]; then
    echo " ! No 'grep' Command."
    exit 1
fi
if [ ! -x /usr/bin/wc ]; then
    echo " ! No 'wc' Command."
    exit 1
fi

#自身のコマンド名を変数に入れる。usage用
CMDNAME=`basename $0`

#引数のチェックと動作フラグの生成
#https://shellscript.sunone.me/parameter.html
while getopts vt OPT
do
    case $OPT in
        "v" )
            #verboseモード
            MODE_VERBOSE=1 ;;
        * )
            #それ以外の指定は、usageを表示
            echo "Usage: $CMDNAME [-v] [-t]" 1>&2
            exit 1 ;;
    esac
done

#起動メッセージ
if [ "$MODE_VERBOSE" = 1 ]; then
  #メッセージ
  echo " * Monitor the ROS Process"
fi

#ポートの初期化
`/usr/local/bin/gpio -g mode ${SET_LAMP_PORT} out`

#直前のステータス
STAT_PORT_PREV=0

#監視ループ
while :
do
    #ROS2の状態を問い合わせ
    #文字長さを数値で取得
    STAT_PORT_NOW=$((`/bin/ps -e | /bin/grep ros2 | /usr/bin/wc -m`))

    #条件判定
    if [ ${STAT_PORT_NOW} -eq 0 -a ${STAT_PORT_PREV} -gt 0 ]; then
        #ROS2が見つからない
        #異常ランプ点灯
        `gpio -g write ${SET_LAMP_PORT} 1`
        if [ "$MODE_VERBOSE" = 1 ]; then
            echo 'ROS2 lost'
        fi
    elif [ ${STAT_PORT_NOW} -gt 0 -a ${STAT_PORT_PREV} -eq 0 ]; then
        #ROS2が見つかった
        #異常ランプ消灯
        `gpio -g write ${SET_LAMP_PORT} 0`
        if [ "$MODE_VERBOSE" = 1 ]; then
            echo 'ROS2 OK'
        fi
    fi
    #直前のステータス変数に、現在の状態を代入
    STAT_PORT_PREV=${STAT_PORT_NOW}

    #ウェイト
    sleep ${SET_WATCH_INTERVAL}
done

exit 0
