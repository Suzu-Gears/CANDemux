[English](./README.md) | 日本語

# CANDemux - Arduino向け仮想CANバス分配器(デマルチプレクサ)

1つの物理CANバスを CAN ID によって複数の仮想バスへ振り分けるデマルチプレクサです。arduino::HardwareCAN 上で動作する仮想CANインターフェース（VirtualCAN）を提供し、これらは arduino::HardwareCAN を受け取る既存のコードや API にそのまま渡して使えます。各仮想バスは受信する ID をリストや範囲で指定でき、受信キューサイズやオーバーフローポリシーを設定できます。

## 対応ハードウェア

`CANDemux` は arduino::HardwareCAN インターフェースを提供する CAN コントローラ／ボードで動作します。例:

*   **Arduino (Uno R4 WiFi / Uno R4 Minima / Nano R4):**
    *   ライブラリ: `<Arduino_CAN.h>` (ビルトイン)
    *   チュートリアル: https://docs.arduino.cc/tutorials/uno-r4-minima/can

*   **ESP32:**
    *   ライブラリ: `<ESP32_TWAI.h>` (Arduino IDE ライブラリマネージャーで "`ESP32_TWAI`" を検索)
    *   GitHub: [eyr1n/ESP32_TWAI](https://github.com/eyr1n/ESP32_TWAI)

*   **Raspberry Pi Pico (RP2040/RP2350):**
    *   ライブラリ: `<RP2040PIO_CAN.h>` (Arduino IDE ライブラリマネージャーで "`RP2040PIO_CAN`" を検索)
    *   GitHub: [eyr1n/RP2040PIO_CAN](https://github.com/eyr1n/RP2040PIO_CAN)

## インストール

<!--
### Arduino IDE ライブラリマネージャー

1.  Arduino IDE を開きます。
2.  `スケッチ > ライブラリをインクルード > ライブラリを管理...` に移動します。
3.  "CANDemux" を検索し、最新バージョンをインストールします。
-->

### 手動インストール

#### Arduino IDE の「.ZIPライブラリをインポート」を使用

1.  [GitHubリポジトリ](https://github.com/Suzu-Gears/CANDemux/releases/latest) から最新リリースを `.zip` ファイルでダウンロードします。
2.  Arduino IDE で、`スケッチ > ライブラリをインクルード > .ZIPライブラリをインポート...` に移動します。
3.  ダウンロードした `.zip` ファイルを選択します。
4.  Arduino IDE を再起動します。

#### 直接配置

1.  [GitHubリポジトリ](https://github.com/Suzu-Gears/CANDemux/releases/latest) から最新リリースをダウンロードします。
2.  ダウンロードしたファイルを解凍し、`CANDemux` フォルダをArduinoのライブラリディレクトリ（例: `~/Documents/Arduino/libraries/`）に配置します。
3.  Arduino IDE を再起動します。
