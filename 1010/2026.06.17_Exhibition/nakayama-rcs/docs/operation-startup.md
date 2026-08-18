# MSD700 遠隔操縦システム 起動手順

ローカル環境／外部ネットワーク経由、それぞれの遠隔操縦の起動手順をまとめる。

## 対象マシン

| 役割 | ホスト名 |
|------|----------|
| 遠隔操作PC（操縦側・デスクトップ） | `user@resolute` |
| Jetson（建機側） | `user@agx-orin` |

---

## A. ローカル環境での遠隔操作（同一ネットワーク内）

操縦PCと Jetson が同じネットワークにいる場合。LiveKit サーバーも操縦PC上で起動する。

### 1. livekit-keys の設定（操縦PC・Jetson 両方）

`nakayama-rcs/livekit-keys` の 7 行目を `if true; then` にする。
`if` ブロック内の `livekit_addr` を、操縦PCのインターフェースが接続している IP に設定する。

```bash
if true; then
    livekit_addr=<操縦PCのIPアドレス>
    livekit_api_key=devkey
    livekit_api_secret=secret
fi
```

### 2. LiveKit サーバーの起動（操縦PC）

```bash
cd ~/build/nakayamahd-rcs && distfiles/livekit-server --dev --bind 0.0.0.0
```

### 3. socket（操縦入力）の起動（操縦PC）

```bash
~/Desktop/t16000m.sh
```

### 4. カメラ映像の起動（操縦PC）

```bash
~/Desktop/all.sh
```

---

## B. 外部ネットワーク経由での遠隔操作

操縦PCと Jetson が別ネットワークにいる場合。LiveKit サーバーは AWS 上の
遠隔操作サーバー（`18.181.7.124`）を使用する。

> **ローカルとの違い**: AWS サーバーを使うため、**操縦PCでの LiveKit サーバー起動（A の手順2）は行わない。**

### 1. 遠隔操作サーバーの起動確認

サーバーは毎晩 22:00 に自動停止する（後述）。停止していれば起動してから、
到達できることを確認する。

```bash
curl http://18.181.7.124:7880
```

`OK` が返れば接続可能。

### 2. livekit-keys の設定（操縦PC・Jetson 両方）

`nakayama-rcs/livekit-keys` の 7 行目を `if false; then` にする。
これにより上段（AWS サーバー）の設定が使用される。上段の値は変更しない。

```bash
livekit_addr=18.181.7.124
livekit_port=7880
livekit_api_key=APIpFNYmcQKHFrx
livekit_api_secret=<シークレット>

if false; then          # ← ここを false にする
    ...
fi
```

### 3. socket（操縦入力）の起動（操縦PC）

```bash
~/Desktop/t16000m.sh
```

### 4. カメラ映像の起動（操縦PC）

```bash
~/Desktop/all.sh
```

手順 3・4 はローカルと共通。

---

## 遠隔操作サーバーについて

| 項目 | 値 |
|------|-----|
| インスタンス ID | `i-0c935c17062ec627b` |
| 名称 | `nakayama-remote-server` |
| アドレス | `18.181.7.124` |
| ポート | `7880` |

- **毎晩 22:00 に自動停止する設定**になっている。
- 使用する際は、その都度サーバーを起動すること。

> 【要確認】起動方法（AWS マネジメントコンソール／AWS CLI／管理者へ依頼 など）を追記する。

---

## トラブルシューティング

- **接続時に認証エラーが出る**
  → サーバー側とクライアント側の API キー／シークレットが食い違っている可能性。
  AWS サーバーが `devkey`/`secret`（`--dev` 起動）で動いている場合は、
  livekit-keys 上段も `devkey`/`secret` に合わせる必要がある。

- **接続はできるが映像が出ない**
  → 映像は UDP ポートを使用する。`curl`（TCP 7880）が通っても、
  サーバーのセキュリティグループで UDP の映像ポートが開いていないと映像が届かない。
  管理者に UDP ポートの開放状況を確認する。

- **展示会場など別ネットワークから接続できない**
  → セキュリティグループが接続元 IP を制限している可能性。
  会場のネットワークからの接続を許可してもらう必要がある。
