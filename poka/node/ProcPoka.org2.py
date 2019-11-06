#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# 棚ボタンの制御
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

from time import sleep


_RUN = 1
_PAUSE = 0


class Proc_Poka(Node):
    """
    棚ボタンの制御
    """

    # ノード名
    SELFNODE = "proc_poka"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE

    # 点灯パターン
    __pattern = [1, 3, 5, 3, 5, 1]

    # 運転ステータス
    __status = 0
    # シーケンス
    __sequence = 0
    # 点灯パターンの進捗
    __sequence_pattern = 0
    # シーケンス中のタイマー
    __sequence_timer_1 = 0
    __sequence_timer_2 = 0
    __sequence_timer_3 = 0

    # 最後に押されたボタン
    __last_button = -1

    def __init__(self):
        '''
        初期化
        '''
        # ノードの初期化
        super().__init__(self.SELFNODE)
        self.get_logger().info("%s initializing ..." % (self.SELFNODE))
        # メッセージの受信・Console
        self.sub_console = self.create_subscription(
            ConsoleMsg, "mes_proc_console", self.callback_console, 10)
        # メッセージの受信・I2C
        self.sub_i2c = self.create_subscription(
            GpioMsg, "mes_gpio_i2c_in", self.callback_i2c, 10)
        # メッセージの送信・GPIO
        self.pub_gpio = self.create_publisher(
            GpioMsg, "mes_gpio_out", 10)
        # メッセージの送信・I2C
        self.pub_i2c = self.create_publisher(
            GpioMsg, "mes_gpio_i2c_out", 10)
        # タイマーのインスタンスを生成（0.2秒ごとに発生）
        self.create_timer(0.2, self.callback_timer)
        # ランプを消す
        self.lamp_con_gpio(9, 0)
        self.lamp_con_i2c(9 + 200, 0)
        # リモコンの更新
        self.update_console()
        # 立ち上げ完了
        self.get_logger().info("%s do..." % self.SELFNODE)

    def __del__(self):
        """
        デストラクタ
        """
        # ランプを消す
        self.lamp_con_gpio(9, 0)
        self.lamp_con_i2c(9 + 200, 0)
        # 処理完了
        self.get_logger().info("%s done." % self.SELFNODE)

    def callback_console(self, message):
        """
        コールバック関数・メッセージの受信
        Parameters
        ----------
        message : ConsoleMsg
            メッセージ
        """
        pass
        # 運転ステータスの更新
        if message.start == 0:
            # 停止
            self.__status = _PAUSE
            self.get_logger().info(' > PAUSE')
        elif message.start == 1:
            # 再開
            self.__status = _RUN
            self.get_logger().info(' > RUN')
        # シーケンスリセット
        if message.start == 0 and message.reset == 1:
            # シーケンスの数字を0にする
            self.__sequence = 0
            self.__sequence_pattern = 0
            # ランプ全消灯
            self.lamp_con_i2c(9 + 200, 0)
            self.get_logger().info(' > SEQUENCE RESET')
        # メニュー切替
        if message.start == 0 and message.cursor_up == 1:
            # カーソル・アップ
            self.get_logger().info(' > CURSOR UP')
            self.__sequence = 0
            self.__sequence_pattern = 0
        elif message.start == 0 and message.cursor_down == 1:
            # カーソル・ダウン
            self.get_logger().info(' > CURSOR DOWN')
            self.__sequence = 0
            self.__sequence_pattern = 0
        # リモコンの更新
        self.update_console()

    def callback_i2c(self, message):
        """
        コールバック関数・I2C・メッセージ受け取り
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        self.get_logger().info(' > I2C: %d V: %d' % (message.port, message.value))
        # 最後に押されたボタンを更新
        if message.value == 1:
            self.__last_button = message.port - 100

    def lamp_con_gpio(self, port, value):
        """
        ランプの点灯制御・GPIO
        ----------
        port : int
            ポート番号
        value : int
            点灯パターン
        """
        # 送信するメッセージの作成
        msg = GpioMsg()
        msg.port = port
        msg.value = value
        # ボタンが押された情報を送信
        self.pub_gpio.publish(msg)

    def lamp_con_i2c(self, port, value):
        """
        ランプの点灯制御・I2C
        ----------
        port : int
            ポート番号
        value : int
            点灯パターン
        """
        # 送信するメッセージの作成
        msg = GpioMsg()
        msg.port = port
        msg.value = value
        # ボタンが押された情報を送信
        self.pub_i2c.publish(msg)

    def callback_timer(self):
        """
        コールバック関数・タイマーの実行部
        """
        pass
        # 運転中で無いときはシーケンスを進めない
        if self.__status != _RUN:
            return
        # 現在の点灯対象のランプ番号
        current_lamp = self.__pattern[self.__sequence_pattern]
        # -------------------------------
        # メインシーケンス
        # -------------------------------
        # シーケンス：初期処理
        if self.__sequence == 0:
            # シーケンスを進める
            self.__sequence += 1
            pass
        # シーケンス：
        elif self.__sequence == 1:
            # 対象のランプを点滅
            self.lamp_con_i2c(current_lamp + 200, 3)
            # シーケンスを進める
            self.__sequence += 1
            pass
        # シーケンス：
        elif self.__sequence == 2:
            # 対象のボタンの入力待ち
            if self.__last_button > 0:
                if self.__last_button == current_lamp:
                    # 該当のボタンを押したときは次のシーケンスに
                    self.__sequence += 1
                else:
                    # 間違ったボタンを押したときはエラーシーケンスに
                    self.__sequence = 51
                # ボタンの内容をリセット
                self.__last_button = -1
            pass
        # シーケンス：
        elif self.__sequence == 3:
            # 対象のボタンのランプを点灯
            self.lamp_con_i2c(current_lamp + 200, 1)
            # シーケンスを進める
            self.__sequence = 98
            pass
        # シーケンス：
        elif self.__sequence == 4:
            # シーケンスを進める
            self.__sequence += 1
            pass
        # シーケンス：終了前処理
        elif self.__sequence == 98:
            # パターンをインクリメント
            self.__sequence_pattern += 1
            if self.__sequence_pattern >= len(self.__pattern):
                # パターン長を超えたらリセット
                self.__sequence_pattern = 0
                # ランプ全消灯
                self.lamp_con_i2c(9 + 200, 0)
            # シーケンスを進める
            self.__sequence += 1
            pass
        # シーケンス：終了
        elif self.__sequence == 99:
            self.__sequence = 0
            pass
        # -------------------------------
        # エラーシーケンス
        # -------------------------------
        # シーケンス：
        elif self.__sequence == 51:
            # 間違ったボタンを押したら、正しいボタンを高速点滅
            self.lamp_con_i2c(current_lamp + 200, 4)
            self.__sequence = 2
            pass
        # 未定義のシーケンス番号を拾った
        else:
            self.get_logger().error(' > NO SEQUENCE !! : %s' % (self.__sequence))
            sleep(10)
            pass
        # ログの表示
        self.get_logger().info(' > __sequence_pattern: %d __sequence: %d' %
                               (self.__sequence_pattern, self.__sequence))

    def update_console(self):
        """
        リモコンのランプ更新
        """
        if self.__status == _RUN:
            # RUN状態の時
            # STARTボタン
            self.lamp_con_gpio(0, 1)
            # STOPボタン
            self.lamp_con_gpio(1, 0)
        else:
            # RUN以外の時
            # STARTボタン
            self.lamp_con_gpio(0, 0)
            if self.__sequence == 0:
                # シーケンスリセット状態の時
                # STOPボタン
                self.lamp_con_gpio(1, 1)
            else:
                # STOPボタン
                self.lamp_con_gpio(1, 3)


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
        node = Proc_Poka()

        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
