#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# GPIO In
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import rclpy
import datetime
from rclpy.node import Node
#import pigpio
import wiringpi
# ローカル
import node.conf as c
#import MyStat as s
# カスタムメッセージ
from poka_mes.msg import GpioMsg


class GpioIn(Node):
    """
    GPIO出力
    """

    # ノード名
    SELFNODE = "gpio_in"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE
    # GPIO入力対象のBCM番号
    __GpioPin = []
    # GPIO入力・前回の値
    __GpioPin_before = []
    # チャタリング除去時間(us)
    __chattering_cancel = 1000
    # 前回コールバック時刻
    __now_before = datetime.datetime.today()
    # 長押し検知時間(us)
    __nagaoshi = 1000000
    # GPIO入力・長押し開始時刻
    __nagaoshi__GpioPin_before = []

    def __init__(self, argPin, arg_verbose=False):
        """
        コンストラクタ
        Parameters
        ----------
        argPin : int
            入力ピン番号
        arg_verbose:bool
            メッセージの強制表示
        """

        # ノードの初期化
        super().__init__(self.SELFNODE)
        pins = ','.join(map(str, argPin))
        self.get_logger().info("%s initializing (%s)..." % (self.SELFNODE, pins))
        # メッセージの送信
        self.pub = self.create_publisher(GpioMsg, self.SELFTOPIC, 10)
        # 制御対象のピン番号を受け取り
        self.__GpioPin = argPin
        # WiringPiの初期化
        self.WiringPiInit()
        # タイマーのインスタンスを生成：Xの定期読み込み用（0.1秒ごとに発生）
        #self.create_timer(10.0, self.in_update_thread)
        # 立ち上げ完了
        self.get_logger().info("%s do..." % self.SELFNODE)

    def __del__(self):
        """
        デストラクタ
        """
        self.get_logger().info("%s done." % self.SELFNODE)

    def WiringPiInit(self):
        '''
        WiringPiの初期化
        '''
        # BCMモードで初期化
        wiringpi.wiringPiSetupGpio()
        # BCMのn番ピンを入力・プルアップに設定
        for i in self.__GpioPin:
            # IoExpICのINT出力監視先ポートの設定
            wiringpi.pinMode(i, wiringpi.GPIO.INPUT)
            # プルアップの設定
            wiringpi.pullUpDnControl(i, wiringpi.GPIO.PUD_UP)
            # コールバック:
            wiringpi.wiringPiISR(
            #    i, wiringpi.GPIO.INT_EDGE_RISING, self.callback_in_rising)
                i, wiringpi.GPIO.INT_EDGE_BOTH, self.callback_in_rising)
            #前回の値初期化
            self.__GpioPin_before.append(0)
            #長押し開始時刻初期化
            self.__nagaoshi__GpioPin_before.append(datetime.datetime.today())

    def callback_in_rising(self):
        """
        スイッチ押下のコールバック関数
        """
        # チャタリング除去処理
        # 現在時刻
        now = datetime.datetime.today()

        # チャタリング無視時刻を超えていたら
        if now > self.__now_before + datetime.timedelta(microseconds=self.__chattering_cancel):
            # GPIO読み込み
            self.in_update_thread()
        # 前回の時刻を更新
        self.__now_before = now

    def in_update_thread(self, arg_val=1):
        """
        GPIO入力の定期読み込み
        Parameters
        ----------
        arg_val
            検出する値(L:0/H:1)
        """
        # 現在のH/L状態の一時格納
        val_now = []
        # 送信するメッセージの作成
        msg = GpioMsg()
        # 指定の入力ピンを一通りスキャン
        for i in self.__GpioPin:
            # BCMポート番号をX端子番号に読み替え
            msg.port = self.__GpioPin.index(i)
            #現在値を読み込み
            if wiringpi.digitalRead(i) == 0:
                msg.value = 0
                #前回と値が違うときだけ処理
                if self.__GpioPin_before[msg.port] != msg.value:
                    # ボタンのON/OFF情報を送信
                    self.pub.publish(msg)
                    self.__GpioPin_before[msg.port] = msg.value
            else:
                msg.value = 1
                #前回と値が違うときだけ処理
                if self.__GpioPin_before[msg.port] != msg.value:
                    # ボタンのON/OFF情報を送信
                    self.pub.publish(msg)
                    self.__GpioPin_before[msg.port] = msg.value
            # 現在のH/L状態に追記
            val_now.append(msg.value)
        # ログの表示
        self.get_logger().info("Port (IN): %s -> %s" % (self.__GpioPin, val_now))
        val_now.clear()


def main(args=None):
    """
    メイン関数
    Parameters
    ----------
    """
    try:
        # rclpyの初期化
        rclpy.init(args=args)
        # インスタンスを生成
        node = GpioIn(c.GPIO_IN)
        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
