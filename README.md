# cugo_pico_encoder_control

Raspberry Pi Pico で CuGo の左右ホイール A/B 相エンコーダを直接読み取り、差動駆動のモータ制御を行う Arduino スケッチです。ROS 側から `/cmd_vel` 相当の `v` (並進速度 [m/s]) と `w` (角速度 [rad/s]) を USB シリアル経由で受信し、Pico 内部で左右ホイールの目標 RPM に変換して PID 制御します。`cugo_ros_motorcontroller` (Apache License 2.0) をベースに Raspberry Pi Pico 向けへ移植した派生物であり、同ライセンスの条件に従って配布しています。

## 主な機能
- A/B 相エンコーダを割り込みで監視し、前後進を問わずカウントを積算
- MotorController(PID + 低域通過フィルタ) により左右ホイールの回転数を安定化、PWM + 方向ピンでモータドライバを直接駆動
- PacketSerial ベースのバイナリプロトコルで PC から目標 RPM を受信し、最新カウントを返信
- 0.5 秒間通信が途絶すると自動停止するフェイルセーフ

## 必要環境
- Raspberry Pi Pico (Arduino Mbed OS RP2040 core を想定)
- PWM 入力に対応したモータドライバ (H ブリッジ / Phase-Enable など) ※想定ハード: Cytron MDDA20A
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
4. Pico を USB 接続し、Arduino IDE の「マイコンボードに書き込む」でビルド・転送します（UF2 を手動でコピーする場合は BOOTSEL ボタンを押したまま接続してください）。CLI の場合は `arduino-cli compile --fqbn rp2040:rp2040:rpipico cugo_pico_encoder_control/cugo_pico_encoder_control` → `arduino-cli upload -p /dev/ttyACM0 --fqbn ...` で書き込めます。
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

--- 
## Raspberry Pi Pico での配線
<img src=docs/pico_pinout.png width=60%>     

### エンコーダ (AMT102-V 例)
Pico 版スケッチでは以下の GPIO 割り当てで配線する想定です ( `cugo_pico_encoder_control/cugo_pico_encoder_control.ino` の `PIN_ENCODER_*` 定数)。

| 信号 | 左ホイール GPIO | 右ホイール GPIO | 備考 |
| --- | --- | --- | --- |
| A相 (黄色 / pin1) | GP3 | GP2 | `PIN_ENCODER_L_A` / `PIN_ENCODER_R_A` |
| B相 (青色 / pin3) | GP9 | GP8 | `PIN_ENCODER_L_B` / `PIN_ENCODER_R_B` |
| Z相 (紫色 / pin4) | 未配線 (NC) | 未配線 (NC) | 本スケッチでは未使用 |
| 5V (橙色 / pin2) | VBUS または外部 5V | VBUS または外部 5V | エンコーダ電源 |
| GND (茶色 / pin5) | 任意の GND | 任意の GND | Pico と共通 GND に接続 |

Pico の GPIO は 3.3V 系なので、AMT102-V など 5V ロジック出力のエンコーダをそのまま接続しないでください。オープンコレクタ設定＋3.3V プルアップ、もしくは[レベルシフタ](https://akizukidenshi.com/catalog/g/g113837/)等で 3.3V 以内に収めてから `GP3/GP9/GP2/GP8` へ入力してください。向きが合わない場合はソース内の `PIN_ENCODER_*` を変更すれば任意の GPIO へ再割り当てできます。

### モータドライバ (PWM/Dir)
`cugo_pico_encoder_control.ino` のデフォルト定数では、モータドライバの PWM / 方向ピンを以下の GPIO に割り当てています。Cytron MDDA20A（デュアル DC モータドライバ、PWM+DIR 方式）を想定した配線です。

| モータ | PWM ピン (`PIN_MOTOR_*_PWM`) | DIR ピン (`PIN_MOTOR_*_DIR`) |
| --- | --- | --- |
| 左車輪 | GP26 | GP16 |
| 右車輪 | GP27 | GP17 |

Phase/Enable 形式のドライバであれば PWM ピンを Enable へ、DIR ピンを Phase へ接続してください。IN/IN 形式の H ブリッジを使う場合は、PWM ピンを片側入力に接続し、もう片側入力を DIR で制御できるよう配線します。別の GPIO を使いたい場合は `.ino` 内の `PIN_MOTOR_*` 定数を書き換えれば対応できます (PWM 可能な GPIO を割り当ててください)。

--- 
## 開発環境メモ

### 開発用 PC / IDE
- Ubuntu 22.04 LTS + VS Code + Arduino 拡張機能 (`arduino-cli` を外部コマンドとして利用)

### Arduino CLI セットアップとビルド
1. 公式バイナリを入手して配置します（Snap 版では 32bit 依存ライブラリを参照できない場合があるため）。
    ```bash
    cd /tmp
    wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz
    tar xf arduino-cli_latest_Linux_64bit.tar.gz
    sudo mv arduino-cli /usr/local/bin/
    ```
2. 初期設定と Philhower 版 RP2040 コアの登録を行います。
    ```bash
    arduino-cli config init
    arduino-cli config set board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
    arduino-cli core update-index
    arduino-cli core install rp2040:rp2040
    ```
3. ライブラリ依存（PacketSerial）を追加します。Servo はボードパッケージ同梱版を利用できます。
    ```bash
    arduino-cli lib install PacketSerial
    ```
4. スケッチをビルド／書き込みします。`XDG_CACHE_HOME=/path/to/cache arduino-cli ...` のように指定すると、キャッシュを任意ディレクトリへ変更可能です。
    ```bash
    arduino-cli compile --fqbn rp2040:rp2040:rpipico cugo_pico_encoder_control/cugo_pico_encoder_control
    arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 cugo_pico_encoder_control/cugo_pico_encoder_control
    ```


### VS Code での Arduino 拡張利用
VS Code 上では Arduino 拡張機能を使って `arduino-cli` を呼び出しています

1. 拡張ビューで "Arduino" (Microsoft) をインストールします。
2. `.vscode/settings.json` に以下の設定を追加し、CLI パスや FQBN を固定します。
    ```json
    {
      "arduino.path": "/usr/local/bin/arduino-cli",
      "arduino.useArduinoCli": true,
      "arduino.defaultBoard": "rp2040:rp2040:rpipico",
      "arduino.defaultBaudRate": 115200,
      "arduino.defaultPort": "/dev/ttyACM0"
    }
    ```
3. ステータスバーでボード/ポートを選択し、`Arduino: Verify` や `Arduino: Upload` を実行すれば CLI と同じ手順でビルド／書き込みできます。
