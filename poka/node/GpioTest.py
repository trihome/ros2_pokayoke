#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# TEST
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import rclpy
from rclpy.node import Node
# カスタムメッセージ
from poka_mes.msg import GpioMsg


class Gpio_Test(Node):
    """
    GPIO TEST
    """

    # ノード名
    SELFNODE = "gpio_test"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE

    def __init__(self):
        '''
        初期化
        '''
        # ノードの初期化
        super().__init__(self.SELFNODE)
        self.get_logger().info("%s initializing ..." %
                               (self.SELFNODE))
        # メッセージの送信・GPIO
        self.pub_gpio = self.create_publisher(
            GpioMsg, "mes_gpio_out", 10)
        # メッセージの受信・GPIO
        self.sub_gpio = self.create_subscription(
            GpioMsg, "mes_gpio_in", self.callback_gpio, 10)
        # メッセージの送信・I2C
        self.pub_i2c = self.create_publisher(
            GpioMsg, "mes_gpio_i2c_out", 10)
        # メッセージの受信・I2C
        self.sub_i2c = self.create_subscription(
            GpioMsg, "mes_gpio_i2c_in", self.callback_i2c, 10)
        # 立ち上げ完了
        self.get_logger().info("%s do..." % self.SELFNODE)

    def callback_gpio(self, message):
        """
        コールバック関数・GPIO・メッセージ受け取り
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        self.get_logger().info('Incoming request Port (GPIO): %d Value: %d' %
                               (message.port, message.value))
        # 送信するメッセージの作成
        msg = GpioMsg()
        # BCMポート番号をX端子番号に読み替え
        msg.port = message.port
        msg.value = message.value
        # ボタンが押された情報を送信
        self.pub_gpio.publish(msg)

    def callback_i2c(self, message):
        """
        コールバック関数・I2C・メッセージ受け取り
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        self.get_logger().info('Incoming request Port (I2C): %d Value: %d' %
                               (message.port, message.value))
        # 送信するメッセージの作成
        msg = GpioMsg()
        # X端子番号をY端子番号に読み替え（100だけオフセット）
        msg.port = message.port + 100
        msg.value = message.value
        # ボタンが押された情報を送信
        self.pub_i2c.publish(msg)


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
        node = Gpio_Test()

        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
