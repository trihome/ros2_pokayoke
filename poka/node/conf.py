#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Common config file
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

# --------------------------------------
# GPIO入出力設定
# --------------------------------------
# GPIO出力ポート番号
GPIO_OUT = [26, 19, 13, 6, 5, 22]

# GPIO入力ポート番号
GPIO_IN = [21, 20, 16, 12]

# GPIO入力ポート番号
# I2CのINT入力
GPIO_I2CINT_IN = 7


# AD入力
AD_IN = range(0, 7)

# --------------------------------------
# IO機器の入出力割り当て
# --------------------------------------
# GPIO IN/OUT割り当て範囲:作業者状態ボタン
GPIO_OUT_WORKERSTAT = range(0, 4)
GPIO_IN_WORKERSTAT = range(0, 4)
GPIO_WORKERSTATORDER = ["RED", "YELLOW", "GREEN", "BLUE"]

# AD測定間隔(Hz)
GROVE_AD_DETECT_INTERVAL = 2.5

# AD割り当て範囲:監視対象設備の電流センサ
# AD ch 0, 1
GROVE_AD_CT = range(0, 2)
GROVE_AD_CTORDER = ["CT1", "CT2"]

# AD割り当て範囲：監視対象設備のシグナルタワー
# AD ch 2→R, 3→Y, 4→G, 5→B （４灯まで）
GROVE_AD_SIGTOWER = range(2, 5)
GROVE_AD_SIGTOWERORDER = ["RED", "YELLOW", "GREEN", "BLUE"]
# AD ch 2→R, 3→Y, 4→G, 5→B, 6→W　(5灯まで)
# GROVE_AD_SIGTOWER = range(2, 5)
# GROVE_AD_SIGTOWERORDER = ["RED", "YELLOW", "GREEN", "BLUE", "WHITE"]

# GPIO OUT割り当て範囲：本装置のシグナルタワー
GPIO_OUT_SIGTOWER = range(4, 8)
GPIO_SIGTOWERORDER = ["RED", "YELLOW", "GREEN", "BLUE"]


# --------------------------------------
# その他設定：メイン
# --------------------------------------

# 主処理の測定間隔(Hz)
MAIN_INTERVAL = 0.033333
#MAIN_INTERVAL = 0.05

# --------------------------------------
# その他設定：シグナルタワー
# --------------------------------------

# シグナルタワーのランプが点灯と判定する閾値 0-2048
#                          赤    黄    緑    青    白
SIGTOWER_LIGHT_THRESHOLD = (700, 700, 700, 700, 700)

# シグナルタワーの測定時間（秒）
# この秒数測定して、そのデータから点灯・点滅・消灯を判断
SIGTOWER_DETECT_BETWEEN = 6

# シグナルタワーの測定間隔（Hz）
SIGTOWER_DETECT_INTERVAL = 0.05

# --------------------------------------
# その他設定：データベース
# --------------------------------------

# データベースサーバへの書き込み間隔（Hz）
DB_MSSQL_APPEND_INTERVAL = 0.016666
#DB_MSSQL_APPEND_INTERVAL = 0.033333

# SQLiteからSQLServerへ
# 一度にテーブルに追加する上限件数
DB_SQLITE_MSSQL_SEND_MAX = 30

# --------------------------------------
# その他設定：動作監視Mystat
# --------------------------------------

# 測定間隔（Hz）
MYSTAT_DETECT_INTERVAL = 0.1

# 正常以外の状態が発生したときに
# シグナルタワー赤・黄色の点灯を持続する時間(s)
MYSTAT_KEEP_LAMPON_TIME = 120
