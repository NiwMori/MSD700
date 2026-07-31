# msd_arduino ドキュメント

環境展2026 MSD自動作業機（BLV差動駆動 + LINAKシリンダ + BNO055 IMU）
Arduino Mega 2560 ⇄ Jetson AGX Orin (ROS 2 Humble) 間バイナリシリアル通信

---

## 目次

- [structure.md](structure.md) — ディレクトリ構成・各ファイルの役割・ハードウェア対応表・ピンアサイン・LED ステータス表
- [protocol.md](protocol.md) — 通信プロトコル仕様（フレーム構造・メッセージ ID・エラーフラグ・単位早見表・ROS2 トピック一覧）
- [execute.md](execute.md) — セットアップ・実行手順（Arduino 書き込み・udev ルール・ROS2 ノード起動・systemd 自動起動）
- [error-handler.md](error-handler.md) — エラー対処法（通信不通・IMU 未検出・ライブラリ不足・モータ逆回転・udev/systemd トラブル等）
