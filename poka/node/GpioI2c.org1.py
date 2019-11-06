#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# GPIO I2C for IOExpander Board(MCP23017)
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import rclpy
import smbus
import threading
import time
from rclpy.node import Node
# ローカル
import node.conf as c
#import MyStat as s
# カスタムメッセージ
from poka_mes.msg import GpioMsg

CHANNEL = 1  # i2c割り当てチャンネル 1 or 0
ICADDR_DEFAULT = 0x20   # スレーブ側ICアドレス
BEGIN_IN_DEFAULT = 100  # GPIO I2C入力の先頭port番号
BEGIN_OUT_DEFAULT = 200  # GPIO I2C出力の先頭port番号

# MCP23017の割込信号を受け付けるX番号
# conf.py の GPIO_INに設定の
# ポート番号に対応する順番数字
MCP_INT_XNO_DEFAULT = 4

# MCP23017 入出力設定レジスタ
# http://kzhishu.hatenablog.jp/entry/2016/07/19/090000
# https://qazsedcftf.blogspot.com/2019/04/esp32arduinomcp23017.html
# https://www.tnksoft.com/blog/?p=4713
REG_IODIRA = 0x00  # ■■■入出力方向
REG_IODIRB = 0x01  # (0: 出力  1:入力)
REG_IPOLA = 0x02  # ■■■I/O 極性
REG_IPOLB = 0x03  # (0: 0='L', 1='H' ; 1: 1='L', 0='H')
REG_GPINTENA = 0x04  # ■■■状態変化割り込み
REG_GPINTENB = 0x05  # (0:無効 1:有効)
REG_DEFVALA = 0x06  # ■■■状態変化割り込みの規定値
REG_DEFVALB = 0x07  # (この値と逆になったら割り込み発生)
REG_INTCONA = 0x08  # ■■■状態変化割り込みの比較値
REG_INTCONB = 0x09  # (0: 前の値と比較  1:DEFV の値と比較)
REG_IOCONA = 0x0a  # ■■■コンフィグレーションレジスタ※
REG_IOCONB = 0x0b  #
REG_GPPUA = 0x0c  # ■■■プルアップ制御
REG_GPPUB = 0x0d  # (0: プルアップ無効  1:プルアップ有効)
REG_INTFA = 0x0e  # ■■■割込みフラグレジスタ (INTCAP 又は GPIO リードでクリア)
REG_INTFB = 0x0f  # (0: 割り込みなし  1:割り込み発生)
REG_INTCAPA = 0x10  # ■■■割込みキャプチャレジスタ
REG_INTCAPB = 0x11  # (割込み発生時の GPIO の値)
REG_GPIOA = 0x12  # ■■■出力レジスタ
REG_GPIOB = 0x13  # (GPIOの値)
REG_OLATA = 0x14  # ■■■出力ラッチレジスタ
REG_OLATB = 0x15  # (出力ラッチの値)
'''
※IOCON 内容
bit 7     BANK    レジスタのアドレス指定を制御する       
    1 = 各ポートに関連付けられているレジスタが別々のバンクに分かれる
    0 = レジスタが全部同じバンクに入れられる( アドレスが連続した状態)       
bit 6     MIRROR    INT ピンのMirror ビット       
    1 = INT ピン同士を内部接続する
    0 = INT ピン同士を接続しない。
        INTA はPORTA に関連付けられ、INTB はPORTB に関連付けられる。       
bit 5     SEQOP    シーケンシャル動作モードビット       
    1 = シーケンシャル動作が無効になり、アドレスポインタはインクリメントされない
    0 = シーケンシャル動作が有効になり、アドレスポインタがインクリメントされる       
bit 4     DISSLW    SDA 出力のスルーレート制御ビット       
    1 = スルーレートは無効
    0 = スルーレートは有効       
bit 3    HAEN    ハードウェア アドレス イネーブル ビット(MCP23S17 のみ)
    MCP23017は値に関わらず、アドレスピンは常に有効。       
bit 2    ODR    INT ピンをオープンドレイン出力として設定する       
    1 = オープンドレイン出力(INTPOL ビットよりも優先される)
    0 = アクティブ ドライバ出力( 極性はINTPOL ビットで設定する)       
bit 1    INTPOL    このビットでINT 出力ピンの極性を設定する       
    1 = アクティブHigh
    0 = アクティブLow       
bit 0     未実装    0」として読み出し       

尚、BANK を変更すると、アドレスマッピングが変更されるため、注意の事。
( 1バイトアクセスでライトすること。)
'''

# IOExpanderボードのピン配置
REG_IODIRA_IN = 0x00  # 入力設定（1:入力、0:出力）
REG_IODIRB_IN = 0xff  # 入力設定（1:入力、0:出力）


class GpioI2c(Node):
    """
    GPIO I2C
    """
    # ノード名
    SELFNODE = "gpio_i2c"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE
    # I2CExpander ICのアドレス
    __ICADDR = None
    # MCPのアドレス
    __BEGIN_IN_DEFAULT = None
    __BEGIN_OUT_DEFAULT = None
    # 前回の入力状態
    __last = [None] * 8

    def __init__(self, arg_icaddr=ICADDR_DEFAULT,
                 arg_begin_in=BEGIN_IN_DEFAULT,
                 arg_begin_out=BEGIN_OUT_DEFAULT,
                 arg_verbose=False):
        '''
        初期化
        '''
        # 定数の設定
        self.__ICADDR = arg_icaddr
        self.__BEGIN_IN_DEFAULT = arg_begin_in
        self.__BEGIN_OUT_DEFAULT = arg_begin_out
        # ノードの初期化
        super().__init__(self.SELFNODE)
        self.get_logger().info("%s initializing (X:%s-, Y:%s- )..." %
                               (self.SELFNODE, arg_begin_in, arg_begin_out))
        # メッセージの送信
        self.pub = self.create_publisher(
            GpioMsg, self.SELFTOPIC + "_in", 10)
        # メッセージの受信
        self.sub = self.create_subscription(
            GpioMsg, self.SELFTOPIC + "_out", self.callback_out, 10)
        # メッセージの受信・入力割込
        self.sub = self.create_subscription(
            GpioMsg, "mes_gpio_in", self.callback_in, 10)
        #IoExpander ICの初期化
        self.IcInit()
        #立ち上げ完了
        self.get_logger().info("%s do..." % self.SELFNODE)

    def IcInit(self):
        '''
        IOExpanderICの初期化
        '''
        # I2Cの設定
        self.bus = smbus.SMBus(CHANNEL)

        # IOExpanderの設定
        # PORTA
        self.bus.write_byte_data(
            self.__ICADDR, REG_IOCONA, 0b00000110)  # コンフィグ
        self.bus.write_byte_data(
            self.__ICADDR, REG_IODIRA, REG_IODIRA_IN)  # 入出力
        # PORTB
        self.bus.write_byte_data(
            self.__ICADDR, REG_IOCONB, 0b00000110)  # コンフィグ
        self.bus.write_byte_data(
            self.__ICADDR, REG_IODIRB, REG_IODIRB_IN)  # 入出力
        self.bus.write_byte_data(
            self.__ICADDR, REG_GPPUB, REG_IODIRB_IN)  # プルアップ
        self.bus.write_byte_data(
            self.__ICADDR, REG_GPINTENB, REG_IODIRB_IN)  # 割込

    def callback_out(self, message):
        """
        コールバック関数・出力メッセージ受け取り
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
        # ポート番号を計算
        port = arg_port - self.__BEGIN_OUT_DEFAULT
        if (0 <= port) and (port <= 7):
            # 受け取ったポート番号が、範囲を超えてないこと
            if arg_val == 1:
                # 指定の番号をON
                val = 0x01 << port
                # 現在のON場所を読み込み
                current = self.bus.read_byte_data(self.__ICADDR, REG_GPIOA)
                # 制御する箇所をORして作る
                control = current | val
            else:
                # 指定の番号をOFF
                val = 0x01 << port
                # 現在のON場所を読み込み
                current = self.bus.read_byte_data(self.__ICADDR, REG_GPIOA)
                # 制御する箇所をANDして作る
                control = current & ~val
            # ON場所を更新
            self.bus.write_byte_data(self.__ICADDR, REG_OLATA, control)
        elif port == 9:
            # ポート*9番を指定されたときは、全ポートを同時操作
            if arg_val == 1:
                # 全てON
                self.bus.write_byte_data(self.__ICADDR, REG_OLATA, 0xff)
            else:
                # 全てOFF
                self.bus.write_byte_data(self.__ICADDR, REG_OLATA, 0x00)
        else:
            # それ以外の時はエラー
            #self.__s.message("Port %s is not found." % (arg_port), s.WARN)
            self.get_logger().error("Port %s is not found." % (arg_port))

    def callback_in(self, message):
        """
        コールバック関数・割込入力メッセージ受け取り
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        self.get_logger().info('Incoming request Port: %d Value: %d' %
                            (message.port, message.value))
        if message.port == MCP_INT_XNO_DEFAULT and message.value == 0:
            #Val = 1（ボタンが押された）ときだけ処理
            # 入力の読み込み
            self.in_update()

    def in_update(self, arg_int=True):
        """
        GPIO入力
        """
        # GPIO読み込み
        val = None
        if arg_int:
            # 割込発生時点のGPIO
            val = self.bus.read_byte_data(self.__ICADDR, REG_INTCAPB)
            self.get_logger().info("I2C INTCAP: %s" % (bin(val)))

            for i in range(0, 8):
                # ON状態のチェック
                if (val >> i) & 0x01 == 1:
                    level = 1
                else:
                    level = 0
                # X番号に置き換え
                gpio_pin = self.__BEGIN_IN_DEFAULT + i

                # ログの表示
                self.get_logger().info("Port (IN): %s, Val: %s" %
                                    (gpio_pin, level))
                # 送信するメッセージの作成
                msg = GpioMsg()
                # BCMポート番号をX端子番号に読み替え
                msg.port = gpio_pin
                msg.value = level
                # ボタンが押された情報を送信
                self.pub.publish(msg)
        else:
            # 現時点のGPIO
            val = self.bus.read_byte_data(self.__ICADDR, REG_GPIOB)
            self.get_logger().info("I2C GPIO  : %s" % (bin(val)))

    def in_update_thread(self):
        """
        GPIO入力の定期読み込み
        （INTAの信号出力がロックされたままになったときに、
        GPIOを強制的に読み込んで解除）
        pigpioの不具合？によるもののため、暫定的な措置
        """
        while True:
            #GPIOをカラ読み込み
            self.in_update(False)
            #2秒待ち
            time.sleep(2.0)


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
        node = GpioI2c()
        # Xの定期読み込み
        th1 = threading.Thread(target=node.in_update_thread)
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
