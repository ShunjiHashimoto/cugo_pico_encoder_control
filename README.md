# cugo_pico_encoder_control

Raspberry Pi Pico で CuGo の左右ホイール A/B 相エンコーダを直接読み取り、差動駆動のモータ制御を行う Arduino スケッチです。ROS 側から `/cmd_vel` 相当の `v` (並進速度 [m/s]) と `w` (角速度 [rad/s]) を USB シリアル経由で受信し、Pico 内部で左右ホイールの目標 RPM に変換して PID 制御します。`cugo_ros_motorcontroller` (Apache License 2.0) をベースに Raspberry Pi Pico 向けへ移植した派生物であり、同ライセンスの条件に従って配布しています。

## 主な機能
- A/B 相エンコーダを割り込みで監視し、前後進を問わずカウントを積算
- MotorController(PID + 低域通過フィルタ) により左右ホイールの回転数を安定化、PWM + 方向ピンでモータドライバを直接駆動
- PacketSerial ベースのバイナリプロトコルで PC から目標 RPM を受信し、最新カウントを返信
- 0.5 秒間通信が途絶すると自動停止するフェイルセーフ

## 必要環境
- Raspberry Pi Pico (Arduino Mbed OS RP2040 core を想定)
- PWM 入力に対応したモータドライバ (H ブリッジ / Phase-Enable など)
- 左右ホイールの 2 相エンコーダ (A 相を GPIO 2/3, B 相を GPIO 8/9 などに接続)
- USB ケーブル (PC と Pico を接続)

使用ライブラリ: Servo, PacketSerial

## ファイル構成
```
cugo_pico_encoder_control/
├── README.md
├── LICENSE
└── cugo_pico_encoder_control/
    ├── MotorController.cpp
    ├── MotorController.h
    └── cugo_pico_encoder_control.ino
```

## 使い方
1. Arduino IDE (または `arduino-cli`) に「Raspberry Pi Pico / RP2040 by Earle Philhower, III」をインストールし、ボードとして Raspberry Pi Pico を選択します。
2. `cugo_pico_encoder_control/cugo_pico_encoder_control/cugo_pico_encoder_control.ino` を開き、`PIN_*` 定数や PID ゲイン / エンコーダ分解能、車体諸元（ホイール半径 / トレッド / 減速比）が実機と異なる場合は調整してください。
3. 動作確認の段階に応じて `TEST_STAGE` マクロを設定できます。`TEST_STAGE=1` でエンコーダカウントのみシリアルへ出力、`TEST_STAGE=2` で一定RPMを指示して MotorController の挙動確認、`TEST_STAGE=3`（デフォルト）で PacketSerial 入出力を使用します。
4. Pico を USB 接続し、Arduino IDE の「マイコンボードに書き込む」でビルド・転送します（UF2 を手動でコピーする場合は BOOTSEL ボタンを押したまま接続してください）。CLI の場合は `arduino-cli compile --fqbn arduino-pico:rp2040:pico cugo_pico_encoder_control/cugo_pico_encoder_control` → `arduino-cli upload -p /dev/ttyACM0 --fqbn ...` で書き込めます。
5. `TEST_STAGE=1/2` はシリアルモニタ (115200 bps) を使って挙動を確認してください。`TEST_STAGE=3` で Pico に書き込んだら、PC 側の ROS パッケージ (例: `cugo_ros2_control2`) を起動し、USB CDC ポートに PacketSerial フォーマットで `v`/`w` を送信します。Pico は受け取った `v`/`w` から左右目標 RPM を算出して制御し、エンコーダカウントを返信します。

## 通信プロトコル
送受信ともに 8 バイトのヘッダ + 64 バイトのボディで構成します。ヘッダ 6〜7 バイト目はボディのチェックサム (16-bit one's complement)。

- PC→Pico: ボディ 0〜3 バイトに `v` [m/s]、4〜7 バイトに `w` [rad/s] を `float` (LE) で格納してください。Pico 側で左右ホイールの線速度/角速度に変換し、モータ RPM を算出します。
- Pico→PC: ボディ 0〜3 バイトに左エンコーダ、4〜7 バイトに右エンコーダのカウント値 (いずれも `int32_t`) を格納します。

## 補足事項
- 現状は単純な 2 ホイール構成のみ対応しています。RC モードや Ethernet 通信などは実装していません。
- Pico 側の PWM ピン割り当ては使用する ESC に合わせて変更してください (デフォルトは GP26/GP27)。
- パケットを 0.5 秒以上受信しないと自動で停止するため、ホスト側は定期的に指令を送ってください。
- 本ソフトウェアは Apache License 2.0 のもとで配布されています。元プロジェクト (`cugo_ros_motorcontroller`) の著作権表示とライセンス文書は `LICENSE` に含まれており、派生物である本パッケージでも継承しています。

## エンコーダ配線
| pin | function | 配線色 |
| --- | ---      | --- |
|1| A相 | 黄色 |
|2| 5V | 赤色 |
|3| B相 | 青色 |
|4| - | 白色 |
|5| GND | 黒色 |

### Raspberry Pi Pico での配線
Pico 版スケッチでは以下の GPIO 割り当てで配線する想定です ( `cugo_pico_encoder_control/cugo_pico_encoder_control.ino` の `PIN_ENCODER_*` 定数)。

- 左モータ A相 (pin1 / 黄色) → GP2
- 左モータ B相 (pin3 / 青色) → GP8
- 右モータ A相 (pin1 / 黄色) → GP3
- 右モータ B相 (pin3 / 青色) → GP9
- 5V (pin2 / 赤色) → Pico の VBUS (USB 5V) もしくは外部電源から VSYS へ入力している 5V 系
- GND (pin5 / 黒色) → Pico の任意の GND

Pico の GPIO は 3.3V 系なので、AMT102-V など 5V ロジック出力のエンコーダをそのまま接続しないでください。オープンコレクタ設定＋3.3V プルアップ、もしくは[レベルシフタ](https://akizukidenshi.com/catalog/g/g113837/)等で 3.3V 以内に収めてから `GP2/GP8/GP3/GP9` へ入力してください。向きが合わない場合はソース内の `PIN_ENCODER_*` を変更すれば任意の GPIO へ再割り当てできます。

## 開発環境メモ

### 開発用 PC / IDE
- Ubuntu 22.04 LTS + VS Code + Raspberry Pi Pico 拡張機能を使用し、`blink` などのサンプルをビルド。

### picotool の導入
1. ソースを取得。
    ```bash
    git clone https://github.com/raspberrypi/picotool
    cd picotool
    ```
2. 依存をインストール。
    ```bash
    sudo apt install build-essential pkg-config libusb-1.0-0-dev cmake
    ```
3. ビルドとインストール。
    ```bash
    mkdir -p build
    cd build
    cmake ..
    make -j
    sudo cmake --install .
    ```
   - これにより `picotool reboot` を含む USB 関連コマンドが有効化される。

### BOOTSEL を使わずに再書き込みする設定
1. USB stdio を有効化。
    ```cmake
    # blink/CMakeLists.txt
    pico_enable_stdio_usb(blink 1)
    pico_enable_stdio_uart(blink 0)
    ```
2. アプリで `stdio_init_all()` を呼ぶ。
    ```c
    // blink/blink.c
    int main(void) {
        stdio_init_all();
        ...
    }
    ```
3. 書き込みフロー。
    ```bash
    sudo picotool reboot -f -u
    sudo picotool load /path/to/blink.elf -fx
    ```
   - 上記設定により、実行中の Pico が USB CDC (vid:pid 2e8a:000a) として enumerate され、USB の抜き差しなしで再書き込みできます。

### 動作確認
1. ビルドして書き込む。
    ```bash
    cd ~/src/cugo_pico_encoder_control/blink
    cmake -B build -S .
    cmake --build build
    sudo picotool load build/blink.elf -fx
    ```
2. LED の点滅を確認し、続けて BOOTSEL モードへ戻して再書き込みできることを確認。
    ```bash
    sudo picotool reboot -f -u
    sudo picotool load build/blink.elf -fx
    ```
