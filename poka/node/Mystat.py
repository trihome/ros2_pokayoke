#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Monitor the operating status of nodes
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import rclpy
from rclpy.node import Node
#カスタムメッセージ
from poka_mes.msg import GpioMsg


# ----------------------------------
# 定数
# ----------------------------------

# ログメッセージとノードのエラーレベル
NONE = -1  # ノード用
NORMAL = 0  # ノード用
DEBUG = 0  # ログメッセージ用
INFO = 1  # ログメッセージ用
WARN = 2  # 共用
ERROR = 3  # 共用
FATAL = 4  # 共用


# ----------------------------------
# クラス
# ----------------------------------
class MyStat(Node):
    """
    ノードのステータス管理
    （各ノードでインスタンスを作って使用する）
    """
    # ----------------------------------
    # 定数
    # ----------------------------------

    # ----------------------------------
    # ローカル変数
    # ----------------------------------
    __status_now = NONE
    __status_last_debug = None
    __status_last_info = None
    __status_last_warn = None
    __status_last_error = None
    __status_last_fatal = None
    __verbose = False
    # ----------------------------------
    # 公開変数
    # ----------------------------------
    @property
    def __status_now(self):
        """
        ノードの現在ステータス
        """
        return self.__status_now

    @__status_now.setter
    def __status_now(self, value):
        """
        ノードの現在ステータス
        """
        self.__status_now = value

    # ----------------------------------
    # メソッド
    # ----------------------------------

    def __init__(self, arg_parent_node, arg_verbose=False):
        """
        コンストラクタ
        Parameters
        ----------
        arg_parent_node:
            親のrospy
        arg_verbose:bool
            ログメッセージ表示の強制を有効化
        """
        # ステータスの初期化
        self.__status_now = NORMAL
        # ログメッセージ表示を強制する
        self.__verbose = arg_verbose
        # ステータスサービスの登録
        if arg_parent_node is not None:
            self.service = rospy.Service(
                ("srv_%s_stat" % (arg_parent_node)),
                mystat_srv, self.handle)

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def handle(self, mystat_srv):
        """
        問い合わせを受けたらステータスを回答
        Parameters
        ----------
        Returns
        -------
        : mystat_srv
            メッセージ
        """
        # 回答する
        return mystat_srvResponse(
            self.__status_now, self.__status_last_debug, self.__status_last_info,
            self.__status_last_warn, self.__status_last_error, self.__status_last_fatal)

    def message(self, arg_message, arg_level=DEBUG):
        """
        処理メッセージ
        Parameters
        ----------
        arg_message:string
            ログメッセージ
        arg_Type:int
            警告レベル
        """
        # ステータス変更
        self.__status_now = arg_level
        # ログメッセージの加工
        message = ("%s" % (arg_message))
        # ログメッセージの表示
        if arg_level == DEBUG:
            # 現在時刻取得
            self.__status_last_debug = rospy.Time.now()
            if self.__verbose:
                # 画面出力
                rospy.loginfo(message)
            else:
                # 画面出力しない
                rospy.logdebug(message)
        elif arg_level == INFO:
            # 現在時刻取得
            self.__status_last_info = rospy.Time.now()
            # 画面出力
            rospy.loginfo(message)
        elif arg_level == WARN:
            # 現在時刻取得
            self.__status_last_warn = rospy.Time.now()
            # 警告表示
            rospy.logwarn(message)
        elif arg_level == ERROR:
            # 現在時刻取得
            self.__status_last_error = rospy.Time.now()
            # エラー表示
            rospy.logerr(message)
        elif arg_level == FATAL:
            # 現在時刻取得
            self.__status_last_fatal = rospy.Time.now()
            # 致命的エラー
            rospy.logfatal(message)
        else:
            # エラーレベルの設定ミス
            rospy.logwarn(" ! Error level miss")
            pass
