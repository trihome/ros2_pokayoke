#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# GPIO Out
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


class GpioOut(Node):
    """
    GPIO出力
    """

    # ノード名
    SELFNODE = "gpio_out"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE
    # GPIO出力対象のBCM番号
    __GpioPin = []
    __PiGpio = None
    # GPIOの出力ステータス
    # 0:消灯、1:点灯、2:点滅（長）、3:点滅（中）、4:点滅（短）
    __GpioStatus = []
    # 点滅速度の基準変数
    __blink = 0

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
        # メッセージの受信
        self.sub = self.create_subscription(
            GpioMsg, self.SELFTOPIC, self.callback, 10)
        # タイマーのインスタンスを生成（0.25秒ごとに発生）
        self.create_timer(0.25, self.callback_timer)
        # pigpioの初期化
        self.__PiGpio = pigpio.pi()
        if not self.__PiGpio.connected:
            # pigpioに接続できないとき
            self.get_logger().error("pigpio cannnot connect")
            return
        # 制御対象のピン番号を受け取り
        self.__GpioPin = argPin
        # 制御対象のピン番号のステータスを初期化
        for i in self.__GpioPin:
            self.__GpioStatus.append(0)
        # PIGPIOの初期化
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
        # BCMのn番ピンを出力に設定
        for i in self.__GpioPin:
            self.__PiGpio.set_mode(i, pigpio.OUTPUT)
            # 出力状態をLOWに設定
            self.__PiGpio.write(i, 0)

    def callback(self, message):
        """
        コールバック関数
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        self.get_logger().info('Incoming request Port: %d Value: %d' %
                               (message.port, message.value))
        # 出力の更新
        self.out_update(message.port, message.value)

    def callback_timer(self):
        """
        タイマーの実行部
        """
        # 点滅パターン計算
        # 毎回
        pattern_1 = self.__blink >> 0 & 0b1
        # 2回に一回
        pattern_2 = self.__blink >> 1 & 0b1
        # 4回に一回
        pattern_3 = self.__blink >> 2 & 0b1

        # 長点滅の処理
        if pattern_3 == 1:
            for i in range(len(self.__GpioPin)):
                if self.__GpioStatus[i] == 2:
                    self.__PiGpio.write(self.__GpioPin[i], 1)
        else:
            for i in range(len(self.__GpioPin)):
                if self.__GpioStatus[i] == 2:
                    self.__PiGpio.write(self.__GpioPin[i], 0)

        # 中点滅の処理
        if pattern_2 == 1:
            for i in range(len(self.__GpioPin)):
                if self.__GpioStatus[i] == 3:
                    self.__PiGpio.write(self.__GpioPin[i], 1)
        else:
            for i in range(len(self.__GpioPin)):
                if self.__GpioStatus[i] == 3:
                    self.__PiGpio.write(self.__GpioPin[i], 0)

        # 短点滅の処理
        if pattern_1 == 1:
            for i in range(len(self.__GpioPin)):
                if self.__GpioStatus[i] == 4:
                    self.__PiGpio.write(self.__GpioPin[i], 1)
        else:
            for i in range(len(self.__GpioPin)):
                if self.__GpioStatus[i] == 4:
                    self.__PiGpio.write(self.__GpioPin[i], 0)

        # 点滅カウンタ
        self.__blink += 1
        if self.__blink > 7:
            self.__blink = 0

    def out_update(self, arg_port, arg_val):
        """
        GPIO出力
        Parameters
        ----------
        arg_port : 
            ポート
        arg_val : 
            値
        """
        if arg_port < len(self.__GpioPin):
            # 受け取ったポート番号が、配列長を超えていないこと
            if arg_val == 0:
                # 指定の番号をOFF
                self.__PiGpio.write(self.__GpioPin[arg_port], 0)
                # 点灯ステータスの変更
                self.__GpioStatus[arg_port] = 0
            elif arg_val == 1:
                # 指定の番号をON
                self.__PiGpio.write(self.__GpioPin[arg_port], 1)
                # 点灯ステータスの変更
                self.__GpioStatus[arg_port] = 1
            elif arg_val == 2:
                # 指定の番号を点滅・長
                # 点灯ステータスの変更
                self.__GpioStatus[arg_port] = 2
                pass
            elif arg_val == 3:
                # 指定の番号を点滅・中
                # 点灯ステータスの変更
                self.__GpioStatus[arg_port] = 3
                pass
            elif arg_val == 4:
                # 指定の番号を点滅・短
                self.__GpioStatus[arg_port] = 4
                pass
            else:
                pass
        elif arg_port == 99:
            # ポート99番を指定されたときは、全ポートを同時操作
            for i in self.__GpioPin:
                if arg_val == 1:
                    # 全てON
                    self.__PiGpio.write(i, 1)
                else:
                    # 全てOFF
                    self.__PiGpio.write(i, 0)
        else:
            # それ以外の時はエラー
            #self.__s.message("Port %s is not found." % (arg_port), s.WARN)
            self.get_logger().error("Port %s is not found." % (arg_port))


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
        node = GpioOut(c.GPIO_OUT)
        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
