#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# コンソールとステータスの制御
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import rclpy
import time
from rclpy.node import Node
# カスタムメッセージ
from poka_mes.msg import GpioMsg
from poka_mes.msg import ConsoleMsg


class Proc_Console(Node):
    """
    コンソール制御
    """

    # ノード名
    SELFNODE = "proc_console"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE

    # START/STOP状態保持
    __isStart = 0
    # 最後にSTOPが押された時刻
    __timeStop = 0
    # 長押し判定の秒数
    PUSH_LONG = 0.8

    def __init__(self):
        '''
        初期化
        '''
        # ノードの初期化
        super().__init__(self.SELFNODE)
        self.get_logger().info("%s initializing ..." %
                               (self.SELFNODE))
        # メッセージの受信・GPIO
        self.sub_gpio = self.create_subscription(
            GpioMsg, "mes_gpio_in", self.callback_status, 10)
        # メッセージの送信・Console
        self.pub_console = self.create_publisher(
            ConsoleMsg, self.SELFTOPIC, 10)
        # 立ち上げ完了
        self.get_logger().info("%s do..." % self.SELFNODE)

    def __del__(self):
        """
        デストラクタ
        """
        self.get_logger().info("%s done." % self.SELFNODE)

    def callback_status(self, message):
        """
        コールバック関数・GPIO入力受け取り
        Parameters
        ----------
        message : GpioMsg
            メッセージ
        """
        # 送信するメッセージの作成
        msg = ConsoleMsg()
        # ボタンを押したのを検知して、対応するボタンの値をmsgに代入
        if message.value == 1:
            # スイッチ・START/STOP
            if message.port == 0:
                self.__isStart = 1
            elif message.port == 1:
                self.__isStart = 0
                # STOP長押し検出
                self.__timeStop = time.time()
            msg.start = self.__isStart
            # スイッチ・カーソル
            msg.cursor_up = 1 if message.port == 2 else 0
            msg.cursor_down = 1 if message.port == 3 else 0
            # メッセージ送信
            self.pub_console.publish(msg)
            # ログの表示
            print(msg, flush=True)

        # ボタンを離したのを検知して、対応するボタンの値をmsgに代入
        elif message.value == 0:
            # スイッチ・STOP
            if message.port == 1:
                timeSub = time.time() - self.__timeStop
                # 指定時間以上、長押しして離した
                if timeSub > self.PUSH_LONG:
                    msg.reset = 1
                    # メッセージ送信
                    self.pub_console.publish(msg)
                    # ログの表示
                    #print(msg, flush=True)


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
        node = Proc_Console()

        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
