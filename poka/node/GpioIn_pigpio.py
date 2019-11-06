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
from rclpy.node import Node
import pigpio
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
    # PIGPIO
    __PiGpio = None
    # コールバック配列
    __cb = []
    # 前回の入力受付時刻
    __last = [None] * 32
    # チャタリング除去時間(us)
    __chattering_cancel = 5000

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
        self.pub = self.create_publisher(
            GpioMsg, self.SELFTOPIC, 10)
        # pigpioの初期化
        self.__PiGpio = pigpio.pi()
        if not self.__PiGpio.connected:
            # pigpioに接続できないとき
            self.get_logger().error("pigpio cannnot connect")
            return
        #制御対象のピン番号を受け取り
        self.__GpioPin = argPin
        #PIGPIOの初期化
        self.initGpio()
        self.get_logger().info("%s do..." % self.SELFNODE)

    def __del__(self):
        """
        デストラクタ
        """
        self.__PiGpio.stop()
        self.get_logger().info("%s done." % self.SELFNODE)

    def initGpio(self):
        """
        GPIOの初期化
        Parameters
        ----------
        """
        # BCMのn番ピンを入力・プルアップに設定
        for i in self.__GpioPin:
            # 入出力
            self.__PiGpio.set_mode(i, pigpio.INPUT)
            # プルアップ・ダウン
            self.__PiGpio.set_pull_up_down(i, pigpio.PUD_UP)
            # コールバック
            self.__cb.append(self.__PiGpio.callback(
                i, pigpio.EITHER_EDGE, self.callback))

    def callback(self, gpio_pin, level, tick):
        """
        スイッチ押下のコールバック関数
        Parameters
        ----------
        gpio_pin : int
            ピン番号
        """
        #チャタリング無視時間
        diff = self.__chattering_cancel + 1
        # 前回の入力からの時間差を計算
        if self.__last[gpio_pin] is not None:
            diff = pigpio.tickDiff(self.__last[gpio_pin], tick)
        # チャタリング除去・前回入力から指定時間以上の間隔をあけて処理
        if diff > self.__chattering_cancel:
            # ログの表示
            self.get_logger().info("Port (IN): %s, Val: %s, Tick: %s" %
                                    (gpio_pin, level, tick))
            # 送信するメッセージの作成
            msg = GpioMsg()
            # BCMポート番号をX端子番号に読み替え
            msg.port = self.__GpioPin.index(gpio_pin)
            msg.value = level
            # ボタンが押された情報を送信
            self.pub.publish(msg)
        self.__last[gpio_pin] = tick


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
