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
import threading
import time
import datetime
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
    __pattern = [0, 1, 2, 3, 4, 5, 6, 7]
    __pattern_store1 = [1, 3, 5, 7, 0, 2, 4, 6]
    __pattern_store2 = [1, 3, 5, 0, 2, 4]

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
        #self.create_timer(0.2, self.callback_timer)
        # ランプを消す
        self.lamp_con_gpio(99, 0)
        self.lamp_con_i2c(99 + 200, 0)
        sleep(3)
        # リモコンの更新
        self.update_console()
        self.comproc_seqencereset()
        # 立ち上げ完了
        self.get_logger().info("%s do..." % self.SELFNODE)

    def __del__(self):
        """
        デストラクタ
        """
        # ランプを消す
        self.lamp_con_gpio(99, 0)
        self.lamp_con_i2c(99 + 200, 0)
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
            # シーケンスリセット
            self.comproc_seqencereset()
        # メニュー切替
        if message.start == 0 and message.cursor_up == 1:
            # カーソル・アップ
            self.get_logger().info(' > CURSOR UP')
            # シーケンスリセット
            self.comproc_seqencereset()
            # パターンの読み直し
            self.__pattern.clear()
            self.__pattern = self.__pattern_store1[:]
            self.get_logger().info(" > pattern: %s" % self.__pattern)
        elif message.start == 0 and message.cursor_down == 1:
            # カーソル・ダウン
            self.get_logger().info(' > CURSOR DOWN')
            # シーケンスリセット
            self.comproc_seqencereset()
            # パターンの読み直し
            self.__pattern.clear()
            self.__pattern = self.__pattern_store2[:]
            self.get_logger().info(" > pattern: %s" % self.__pattern)
        # リモコンの更新
        self.update_console()

    def comproc_seqencereset(self):
        """
        共通：シーケンスリセット
        """
        # シーケンスの数字を0にする
        self.__sequence = 0
        self.__sequence_pattern = 0
        # ランプ全消灯
        self.lamp_con_i2c(99 + 200, 0)
        self.get_logger().info(' > SEQUENCE RESET')

    def callback_i2c(self, message):
        """
        コールバック関数・I2C・メッセージ受け取り
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        #self.get_logger().info(' > I2C: %d V: %d' % (message.port, message.value))
        # 最後に押されたボタンを更新
        if message.value == 1:
            self.__last_button = message.port - 100
            self.get_logger().info(" > last button: %d" % (self.__last_button))

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
        シーケンス処理
        """
        pass
        # 運転中で無いときはシーケンスを進めない
        if self.__status != _RUN:
            return
        # 現在の点灯対象のランプ番号
        current_lamp = 0
        if self.__sequence_pattern < len(self.__pattern):
            current_lamp = self.__pattern[self.__sequence_pattern]
        else:
            print(" pattern range over (%d)!" % self.__sequence_pattern)
            self.__sequence_pattern = 0

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
            if self.__last_button >= 0:
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
            self.__sequence = 97
            pass
        # シーケンス：
        elif self.__sequence == 4:
            pass
        # シーケンス：終了前処理1
        elif self.__sequence == 97:
            # パターンをインクリメント
            self.__sequence_pattern += 1
            # パターン長を超えたら、全点灯のまま、指定時間点灯のまま保持
            if self.__sequence_pattern >= len(self.__pattern):
                # 現在時刻をシーケンスタイマに代入
                self.__sequence_timer_1 = datetime.datetime.today()
                # シーケンスを進める（１パターン全終了）
                self.__sequence = 98
            # 超えてなければ
            else:
                # シーケンスを進める（１サイクル終了）
                self.__sequence = 99
            pass
        # シーケンス：１パターン全終了処理
        elif self.__sequence == 98:
            # 現在時刻を取得
            now = datetime.datetime.today()
            # 指定時間を超えていたら
            if now > self.__sequence_timer_1 + datetime.timedelta(microseconds=2000000):
                # パターンリセット
                self.__sequence_pattern = 0
                # ランプ全消灯
                self.lamp_con_i2c(99 + 200, 0)
                # シーケンスを進める
                self.__sequence = 99
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
        # self.get_logger().info(' > __sequence_pattern: %d __sequence: %d' %
        #                       (self.__sequence_pattern, self.__sequence))

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

    def thread_seq(self):
        """
        シーケンス処理
        """
        while True:
            # 実行
            self.callback_timer()
            # 待ち
            time.sleep(0.005)


def main(args=None):
    """
    メイン関数
    Parameters
    ----------
    """
    th1 = None
    try:
        # rclpyの初期化
        rclpy.init(args=args)
        # インスタンスを生成
        node = Proc_Poka()
        # シーケンス処理スレッド起動
        th1 = threading.Thread(target=node.thread_seq)
        th1.setDaemon(True)
        th1.start()
        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
