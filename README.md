# cugo_pico_encoder_control

<img src=docs/cugov4_board.png width=50%>

Raspberry Pi Pico で CuGo のエンコーダを読み取り、`/cmd_vel` 相当の `v`/`w` 指令で左右モータを PID 制御する Arduino スケッチです。
`cugo_ros_motorcontroller` (Apache License 2.0) をベースに、Pico 向けへ移植した派生物です。

## 対応機体とスケッチ

| 機体 | スケッチ | 主要な違い |
| --- | --- | --- |
| CuGoV3 | `cugov3_pico_encoder_control/cugov3_pico_encoder_control.ino` | エンコーダ 2相 (A/B)、モータ制御 `PWM + DIR` |
| CuGoV4 | `cugov4_pico_encoder_control/cugov4_pico_encoder_control.ino` | エンコーダ `SPEED-OUT` 単相、モータ制御 `FWD/REV` または `FWDのみ` |

## 主な機能

- 100 Hz（10 ms 周期）の制御ループで、左右モータの目標 RPM を PID + 低域通過フィルタで追従制御し、PWM/方向ピンを直接出力
- エンコーダは割り込みで計測。CuGoV3 は A/B 相（CHANGE 割り込み）、CuGoV4 は `SPEED-OUT` 立ち上がり + DIR ピンで符号付きカウント
- `TEST_STAGE=4` では PacketSerial のバイナリフレームを受信。チェックサム検証後に目標値を反映し、左右エンコーダ値・バッテリー電圧・推定 `v/w` を返信（CuGoV4 は加減速上限も受信してスルーレート制限を適用）
- フェイルセーフは 100 ms 監視 × 5 回（0.5 秒）で発動し、モータ出力を停止。PacketSerial の overflow 検出時も即停止

## 必要環境

- Raspberry Pi Pico（RP2040）
- CuGoV3 または CuGoV4 本体
- モータドライバ
  - CuGoV3: Cytron MDDA20A
  - CuGoV4: BLVD10KM（ブラシレスモータ用）
- 左右ホイールのエンコーダ出力（CuGoV3: A/B 相、CuGoV4: `SPEED-OUT` 単相）
- USB ケーブル（PC と Pico を接続）

メインスケッチ使用ライブラリ: PacketSerial（CuGoV3 では Servo も使用）

## クイックスタート

以下は開発用 PC（`arduino-cli` / Arduino IDE を使う環境）を想定した手順です。
インストールの具体手順は [開発環境・ビルド手順](docs/development.md) の「方法1: ターミナル（Arduino CLI）でセットアップ」を参照してください。
Raspberry Pi 5 上からでも同様にビルド・書き込みできます。詳細は [開発環境・ビルド手順](docs/development.md) の「Quick Start (Raspberry Pi 5)」を参照してください。

1. Arduino IDE または `arduino-cli` を準備し、`Raspberry Pi Pico / RP2040 by Earle Philhower, III` をインストール。
2. ライブラリ `PacketSerial` をインストール。
3. 使用機体に応じて V3/V4 の `.ino` を開く。
4. 必要に応じて以下を実機に合わせて調整。
   - `PIN_*` 定数
   - `kWheelRadius*` / `kTread` / `kReductionRatio`
   - PID ゲイン、エンコーダ分解能
5. Pico へ書き込み。
   ```bash
   arduino-cli compile --fqbn rp2040:rp2040:rpipico cugov3_pico_encoder_control
   arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 cugov3_pico_encoder_control

   arduino-cli compile --fqbn rp2040:rp2040:rpipico cugov4_pico_encoder_control
   arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 cugov4_pico_encoder_control
   ```
6. 通信テスト（PC から指令送信）。
   ```bash
   python3 -m pip install --user pyserial cobs
   python3 scripts/send_test_cmd_vel.py --port /dev/ttyACM0 --v 0.1 --w 0.0 --hz 10
   ```

## TEST_STAGE

| 値 | 用途 |
| --- | --- |
| `1` | エンコーダカウント確認 |
| `2` | 一定 RPM 指令テスト |
| `3` | コード内 `kTestTargetV` / `kTestTargetW` で走行 |
| `4` | PacketSerial による PC 通信（通常運用） |

`TEST_STAGE=3` では `.ino` の `kTestTargetV` / `kTestTargetW` を変更して、Pico 単体で走行確認できます。

## 通信プロトコル

送受信ともに 8 バイトのヘッダ + 64 バイトのボディで構成します。ヘッダ 6〜7 バイト目はボディのチェックサム (16-bit one's complement)。

- PC→Pico: ボディ 0〜3 バイトに `v` [m/s]、4〜7 バイトに `w` [rad/s] を `float` (LE) で格納してください。Pico 側で左右ホイールの線速度/角速度に変換し、モータ RPM を算出します。
- Pico→PC: ボディ 0〜3 バイトに左エンコーダ、4〜7 バイトに右エンコーダのカウント値 (いずれも `int32_t`) を格納します。8〜11 バイトにバッテリー電圧 [V]、12〜15 バイトに `v` [m/s]、16〜19 バイトに `w` [rad/s] を `float` (LE) で格納します。

## ディレクトリ構成

```text
cugo_pico_encoder_control/
├── cugov3_pico_encoder_control/   # CuGoV3 用スケッチ
├── cugov4_pico_encoder_control/   # CuGoV4 用スケッチ
├── scripts/send_test_cmd_vel.py   # シリアル送信テスト用
├── test/                           # 単機能テストスケッチ群
└── docs/                           # 配線・開発手順などの詳細ドキュメント
```

## 詳細ドキュメント

- [配線とピンアサイン](docs/hardware.md)
- [開発環境・ビルド手順](docs/development.md)

## ライセンス

Apache License 2.0。詳細は [LICENSE](LICENSE) を参照してください。
