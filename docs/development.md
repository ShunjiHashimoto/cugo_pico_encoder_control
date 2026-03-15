# 開発環境・ビルド手順

## PC を使った開発環境・ビルド手順

### 前提

- OS 例: Ubuntu 22.04 LTS
- ツール: `arduino-cli`（または Arduino IDE）
- ボードコア: `Raspberry Pi Pico / RP2040 by Earle Philhower, III`

### 方法1: ターミナル（Arduino CLI）でセットアップ

#### セットアップ
1. `arduino-cli` を公式バイナリでインストール（推奨）

    ```bash
    cd /tmp
    wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz
    tar xf arduino-cli_latest_Linux_64bit.tar.gz
    sudo mv arduino-cli /usr/local/bin/
    ```

    補足:
    - Snap 版は環境によって 32bit 依存ライブラリ参照で問題が出る場合があります。

2. 初期設定

    ```bash
    arduino-cli config init
    arduino-cli config set board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
    arduino-cli core update-index
    arduino-cli core install rp2040:rp2040
    ```

3. 依存ライブラリ（メインスケッチ）

    ```bash
    arduino-cli lib install PacketSerial
    ```

    補足:
    - Pico 2 (RP2350) を使う場合は FQBN を `rp2040:rp2040:rpipico2` に変更してください。
    - 利用可能なボードは `arduino-cli board listall rp2040:rp2040` で確認できます。

#### メインスケッチのビルド/書き込み

  ```bash
  # CuGoV3
  arduino-cli compile --fqbn rp2040:rp2040:rpipico cugov3_pico_encoder_control
  arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 cugov3_pico_encoder_control

  # CuGoV4
  arduino-cli compile --fqbn rp2040:rp2040:rpipico cugov4_pico_encoder_control
  arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 cugov4_pico_encoder_control
  ```

必要に応じてキャッシュ保存先を変更できます。

```bash
XDG_CACHE_HOME=/path/to/cache arduino-cli compile --fqbn rp2040:rp2040:rpipico cugov4_pico_encoder_control
```

#### テストスケッチ

`test/motor_pwm_test` と `test/encoder_count_test` は `DRIVER_TYPE` でドライバ種別を切り替えできます。

- `0`: DC
- `1`: BLDC (HM-5100J)
- `2`: BLDC (HP-5097J)

ビルド例:

```bash
# motor_pwm_test
arduino-cli compile --fqbn rp2040:rp2040:rpipico test/motor_pwm_test
arduino-cli compile --fqbn rp2040:rp2040:rpipico --build-property compiler.cpp.extra_flags="-DDRIVER_TYPE=1" test/motor_pwm_test
arduino-cli compile --fqbn rp2040:rp2040:rpipico --build-property compiler.cpp.extra_flags="-DDRIVER_TYPE=2" test/motor_pwm_test

# encoder_count_test
arduino-cli compile --fqbn rp2040:rp2040:rpipico test/encoder_count_test
arduino-cli compile --fqbn rp2040:rp2040:rpipico --build-property compiler.cpp.extra_flags="-DDRIVER_TYPE=1" test/encoder_count_test
arduino-cli compile --fqbn rp2040:rp2040:rpipico --build-property compiler.cpp.extra_flags="-DDRIVER_TYPE=2" test/encoder_count_test
```

--- 

### 方法2: GUI（VS Code Arduino 拡張）でビルド/書き込み

VS Code 上では Arduino 拡張機能を使って `arduino-cli` を呼び出しています。

1. 拡張ビューで `Arduino` (Microsoft) をインストールします。
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

3. ステータスバーでボード/ポートを選択し、`Arduino: Verify` や `Arduino: Upload` を実行すれば、CLI と同じ手順でビルド／書き込みできます。

--- 

## Raspberry Pi を使った開発環境・ビルド手順

1. Arduino CLI をインストール

    ```bash
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    export PATH=$PATH:$HOME/bin
    ```

2. RP2040 コアをインストール

    ```bash
    arduino-cli config add board_manager.additional_urls \
      https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

    arduino-cli core update-index
    arduino-cli core install rp2040:rp2040
    ```

3. 必要ライブラリ

    ```bash
    arduino-cli lib install PacketSerial
    ```

4. リポジトリ取得

    ```bash
    git clone https://github.com/ShunjiHashimoto/cugo_pico_encoder_control.git
    cd cugo_pico_encoder_control
    ```

5. 初回書き込み  
Pico を BOOTSEL を押しながら USB 接続してから実行します。

    ```bash
    arduino-cli compile -u \
      --fqbn rp2040:rp2040:rpipico \
      cugov4_pico_encoder_control
    ```

6. 2回目以降の書き込み

    ```bash
    arduino-cli compile -u \
      -p /dev/ttyACM0 \
      --fqbn rp2040:rp2040:rpipico \
      cugov4_pico_encoder_control
    ```
    Pico のポートは以下で確認できます。

    ```bash
    ls /dev/ttyACM*
    ```
