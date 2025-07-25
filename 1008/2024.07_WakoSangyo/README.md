# MSD700 remote control ROS 2 package
This package includs micro-controlers source cods.

## System Overview
![system overview](figs/全体の通信フロー図.drawio.svg "system overview")

## ドキュメント
- [基本設計書](./Document/基本設計書.md)
- [詳細設計書](./Document/詳細設計書.md)
- [PlatformIOの使い方](./Document/PlatformIOの使い方.md)

### GitHubの使い方について
トピックブランチを採用．
mainブランチは常に動作可能なコードのみにしておく．
追加する機能ごとにブランチを作成して，コード編集，mainへマージしてプルリクエストを行う．
コミットメッセージは，何をしたかを記述．基本日本語で行う．わかりやすければ，英語でも可．commitなど単語は控える．

## Repository
### マシン側
[マシン制御のリポジトリ](https://github.com/SagaUdeLab/msd_remote.git)

### 遠隔端末側
- [遠隔操作クライアントのリポジトリ](https://github.com/SagaUdeLab/msd700-remote-control-suite)
- [コントロールチェアのJoy Stickのリポジトリ](https://github.com/SagaUdeLab/controlchair-joystick)
- [コントロールチェアのリポジトリ](https://github.com/SagaUdeLab/control_chair_motion)