# MA8 — ES7210 ×2 8ch 同時サンプリング レコーダー（ESP32-S3 / ESP32-P4）

ES7210（4ch オーディオADC）を2個使い、**8ch を同時サンプリング**します。取得したデータは次の4通りで取り出せます。

| 出力モード | 内容 |
|---|---|
| **SD** | microSD に 8ch WAV で保存（`/MA8/R0001_00.WAV` …） |
| **SERIAL** | USB-CDC でバイナリパケットを送信。PC側は [tools/ma8_serial_rx.py](tools/ma8_serial_rx.py) で受信して WAV に保存 |
| **USB AUDIO** | 8ch マイクとして認識される（Windows/macOS/Linux 標準ドライバ、Audacity などで録音）。StampS3 は UAC1（Full-Speed）、Stamp-P4 は UAC2（High-Speed） |
| **UART DBG** | デバッグ用。全チャンネルを間引いて（4ch: 約 1kHz、8ch: 約 500Hz）、テキスト（CSV）で USB シリアル（ログと同じポート）へ出力。`UART_G43` を定義するとハードウェア UART（921600bps）へ |

対応モジュールは2種類です。

| | **M5Stack Stamp-P4**（本命） | **M5StampS3**（M5Capsule 内） |
|---|---|---|
| MCU | ESP32-P4（PSRAM 32MB） | ESP32-S3（PSRAM なし） |
| フレームワーク | Arduino 3.x（pioarduino、IDF 5.5） | Arduino 2.0.17（espressif32@6.13.0、IDF 4.4） |
| USB Audio | UAC2 / High-Speed：**8ch・48kHz・32bit まで** | UAC1 / Full-Speed：8ch・16bit・38kHz まで |
| SD | 外付け microSD（SDMMC 4bit） | M5Capsule 内蔵スロット（SPI） |
| 操作 | USB シリアルのコマンド | ボタン＋LED、USB シリアルのコマンド |
| プロジェクト | [p4/platformio.ini](p4/platformio.ini) | [platformio.ini](platformio.ini) |

サンプリングレートとビット数は [include/config.h](include/config.h) の `SAMPLE_RATE_HZ` / `SAMPLE_BITS` で設定します（`-D` で platformio.ini から上書きも可）。

## ビルド

```
# Stamp-P4（初回は pioarduino 一式を ~/.platformio-pioarduino にダウンロードします）
pio run -d p4 -t upload                 # チップ rev v3.0 未満（既定）
pio run -d p4 -e stamp-p4-r3 -t upload  # チップ rev v3.x
pio device monitor -d p4

# StampS3
pio run -e m5stamps3 -t upload
pio device monitor
```

- **Stamp-P4 は別プロジェクト（`p4/`）です。** ソースは共通（`src_dir = ../src`）で、`src/p4/` と `src/s3/` をビルド対象で切り替えます。
- `p4/platformio.ini` は `core_dir = ~/.platformio-pioarduino` を使います。pioarduino は自分用の PlatformIO フォークを core の Python 環境に入れ、共有パッケージも置き換えるため、通常の `~/.platformio`（StampS3 や他のプロジェクトが使う環境）から切り離しています。
- Stamp-P4 のチップリビジョンは書き込み時に esptool が表示します（`Chip is ESP32-P4 (revision vX.Y)`）。v3.0 以上なら `stamp-p4-r3` を使ってください。
- USB は TinyUSB（`ARDUINO_USB_MODE=0`）で動かし、CDC(Serial) と USB Audio の複合デバイスになります。
  - Stamp-P4: 複合デバイスは **USB2 OTG HS** 側に出ます。基板の USB-C は USB Serial/JTAG で、書き込みと起動ログに使います。
  - StampS3: 書き込めないときは、BOOT ボタン(G0)を押したまま USB を挿してから書き込んでください。

---

## 1. 接続設計

### 1.1 構成

```
                    ┌────────── ESP32-P4 / ESP32-S3 ─────────┐
                    │ I2S0 (RX, master, TDM 4slot)            │
 ES7210 A ─SDOUT1──▶│  DIN_A                                  │
 (CH1-4)  ◀─MCLK────│  MCLK  ──┬────────────────────────────▶ ES7210 B MCLK
          ◀─SCLK────│  BCLK  ──┼──┐ (同じパッドを I2S1 の入力にも接続)
          ◀─LRCK────│  LRCK  ──┼──┼───────────────────────▶ ES7210 B SCLK/LRCK
                    │          │  ▼                          │
                    │ I2S1 (RX, slave, TDM 4slot)             │
 ES7210 B ─SDOUT1──▶│  DIN_B                                  │
 (CH5-8)            │                                         │
 A,B ◀──I2C────────▶│  SDA/SCL (400kHz)                       │
                    └─────────────────────────────────────────┘
```

- ESP32 の I2S は、TDM モードでは 1 ポートあたりデータ入力が 1 本しかありません。そこで **I2S0 をマスター**（MCLK/BCLK/LRCK を出力し、ES7210 A のデータを受信）、**I2S1 をスレーブ**（I2S0 と同じ BCLK/LRCK パッドを GPIO マトリクスで入力に引き込み、ES7210 B のデータを受信）にしています。
- 2個の ES7210 は同じ MCLK/SCLK/LRCK で動くため、**サンプリングの瞬間は完全に同期**します。
- ES7210 に渡す LRCK は、GPIO マトリクスで**極性を反転**しています（`ES7210_LRCK_INVERT 1`）。I2S1 は入力側で反転し戻すので、I2S0 と I2S1 のタイミングは揃ったままです。この設定と、スロットの並べ替え `ES7210_SLOT_OF_MIC {0,2,1,3}` の組み合わせで、CH1..4 = A の MIC1..4、CH5..8 = B の MIC1..4 になります（実機で確認済み）。
- 起動時に I2S1 → I2S0 の順で開始し、2つのポートが同じフレームから取り込みを始めるようにしています（詳細と確認方法は「4. 同期の確認」を参照）。
- ES7210 は I2C スレーブとして設定します（TDM、I2S フォーマット）。MCLK は通常 256·Fs です。Stamp-P4 で 24/32bit（スロット 32bit）のときだけ 512·Fs にします（IDF 5 の TDM ドライバは MCLK/BCLK が 2 以下だと MCLK 倍率を自動で変えるため、ES7210 の設定とずれないように先に合わせています）。

### 1.2 ピン割り当て

| 信号 | 方向 | ES7210 ピン | **Stamp-P4** | StampS3（M5Capsule） |
|---|---|---|---|---|
| MCLK | ESP → A, B | 5 MCLK | **G0**（LPG0） | **G1** |
| BCLK (SCLK) | ESP → A, B | 9 SCLK | **G35** | **G5** |
| LRCK | ESP → A, B | 10 LRCK | **G38** | **G7** |
| DIN_A | A → ESP | 11 SDOUT1/TDMOUT (A) | **G17** | **G9** |
| DIN_B | B → ESP | 11 SDOUT1/TDMOUT (B) | **G19** | **G44** |
| SDA | 双方向 | 3 CDATA | **G26** | **G13** (Grove) |
| SCL | ESP → A, B | 4 CCLK | **G27** | **G15** (Grove) |
| UART DBG TX | ESP → PC | — | **G37** | **G43** |
| UART DBG スイッチ | SW → ESP（押すと GND） | — | — | **G0** |
| 3.3V | 電源 | VDDP/VDDD/VDDA/VDDM | 3V3 | 3V3 |
| GND | | GNDD/GNDA/PGND | GND | GND |
| microSD | | — | 外付け SDMMC 4bit：CLK=**G12**, CMD=**G10**, D0=**G8**, D1=**G1**, D2=**G21**, D3=**G23** | M5Capsule 内蔵スロット（CS=G11, SCK=G14, MOSI=G12, MISO=G39） |
| USB（Audio + CDC） | | — | **USB2_HOST_DP / USB2_HOST_DM**（底面パッド）→ 外付け USB コネクタ | 基板の USB-C |

**Stamp-P4 の注意**
- 外部信号には **2.54mm DIP ピン**（G0, G1, G8, G10, G12, G17, G19, G21, G23, G26, G27, G35, G37, G38）だけを使います。例外は USB HS の D+/D- の2本だけです。
- **USB HS（UAC2 / CDC）の配線**: 底面のスタンプパッド `USB2 HOST_DP`（D+）と `USB2 HOST_DM`（D−）を、外付けの USB コネクタ（USB-C ブレークアウトなど）の D+/D− へつなぎ、GND も共通にします。
  - 480Mbps の差動信号なので、**配線はできるだけ短く（数 cm 以内）、D+/D− をより合わせるか平行・等長**にしてください。長いジャンパ線では HS で接続できないことがあります。
  - コネクタの VBUS（+5V）は **5VIN パッド（VIN）** へつなぐのがおすすめです。VIN と基板 USB-C の 5V（`5V_USB_IN`）は、それぞれ理想ダイオード CH213K を通って `SYS_5V` で合流するので、USB-C と HS を同時に挿しても互いに逆流しません。
  - **VBUS パッド（`5V_USB_IN`）** につないでも給電できますが、このネットは基板 USB-C の VBUS と同じで、間には過電圧保護 IC（AW32901）しかありません。その場合は USB-C と同時に挿さないでください（HS 側の 5V には過電圧保護も効きません）。
  - ESP32-P4 の HS デバイスは VBUS 検出を使っていないので、VBUS は給電のためだけの配線です。
- **基板の USB-C は USB Serial/JTAG（G24/G25、Full-Speed）** です。書き込みと起動ログ、コマンド入力に使えます。HS 側をつながなくても立ち上げ確認ができます。
- ストラップピンや特殊なピンには、ESP32 からの出力だけを割り当てています。ES7210 の入力はハイインピーダンスなので、起動時のストラップ値は変わりません。
  - **G35**: BOOT ストラップで、基板に 10k のプルアップがあります → BCLK
  - **G38**: U0RXD → LRCK
  - **G37**: U0TXD → デバッグ UART の TX。リセット直後は ROM と IDF のブートログ（115200bps）が出ます。
  - これらのピンに外付けのプルダウンは付けないでください。
- **G26/G27** は USB Full-Speed OTG の PHY パッドです（USB としては使っていません）。速度の低い I2C に割り当てています。
- **microSD**: レベル変換バッファのない、SD バス（4bit）で使えるソケットを使ってください。SPI 専用の SD モジュールは使えません。CMD と D0〜D3 は 10kΩ で 3.3V にプルアップし、カードは 3.3V で給電します。
- Stamp-P4 にはユーザー用のボタンや LED がありません。操作は USB シリアルのコマンドで行います（「3. 操作」参照）。

**StampS3 の注意**
- 外部信号には 2.54mm ピッチのピン（G1, G3, G5, G7, G9, G13, G15, G40, G43, G44）だけを使います。G3 と G40 は空きです（G3 はストラッピングピン、G40 は M5Capsule 内蔵 PDM マイクのクロックで、ファームウェアではマイクを無効にしています）。
- G43/G44 は UART0 のパッドです。リセット直後は ROM のブートログ（115200bps）が G43 に出ます。
- SD は M5Capsule 内蔵スロットの配線を使います。M5Capsule の電源保持（G46）やボタン（G42）は M5Unified が自動で扱います。

### 1.3 ES7210 周辺回路（各チップ共通）

| ピン | 接続 |
|---|---|
| AD0 / AD1 | **A: AD0=GND, AD1=GND → 0x40**、**B: AD0=3.3V, AD1=GND → 0x41** |
| VDDP, VDDD | 3.3V（各ピンに 0.1µF、近くに 10µF） |
| VDDA, VDDM | 3.3V（できればアナログ専用 LDO またはフェライトビーズ + 10µF/0.1µF で分離） |
| REFP12/REFQ12, REFP34/REFQ34, REFQM | データシートのアプリケーション回路どおりのコンデンサを GNDA へ |
| MICBIAS12, MICBIAS34 | 1µF 程度で GNDA へ。マイクのバイアス電源（前回の回答のマイク回路を参照） |
| MIC1P/N 〜 MIC4P/N | マイク入力（ECM: 2.2kΩ 負荷 + 1µF カップリング、擬似差動） |
| SDOUT2/TDMIN, INT, DMIC_CLK | 未接続 |
| GNDD / GNDA / PGND | GNDA と GNDD はチップ直下で1点接続 |

- MCLK/BCLK/LRCK は2チップに分岐します。配線は短くし、ESP32 の出力側に 22〜33Ω の直列抵抗を入れると反射が抑えられます。
- I2C は 4.7kΩ で 3.3V にプルアップします（モジュール側にプルアップがあれば不要）。
- MICBIAS の電圧（`MIC_BIAS_MV`）と PGA ゲイン（`MIC_GAIN_DB`）は config.h で設定します。

## 2. 設定（include/config.h）

| 定義 | 既定値 | 内容 |
|---|---|---|
| `SAMPLE_RATE_HZ` | 32000 | 8000〜48000 |
| `SAMPLE_BITS` | 16 | 16 / 24（3バイトに詰める）/ 32 |
| `DEFAULT_OUT_MODE` | StampS3: `OUT_MODE_USB_AUDIO` / Stamp-P4: `OUT_MODE_SD` | 起動時のモード |
| `MIC_GAIN_DB` | 30 | PGA ゲイン 0〜37.5dB |
| `MIC_BIAS_MV` | 2870 | MICBIAS 電圧 |
| `FILE_SPLIT_SEC` | 600 | WAV ファイルを分割する間隔（秒） |
| `SD_MMC_KHZ` | 20000 | Stamp-P4 の SD クロック（SDMMC 4bit） |
| `SERIAL_USE_RAW_EP` | S3: 1 / P4: 0 | 1: CDC のバルク IN エンドポイントへ直接転送（S3 用の高速化）／0: Serial.write（P4 は HS なのでこれで十分） |
| `DEBUG_CH` | `NUM_CH` | UART DBG で 1 行に出すチャンネル数（8ch ビルドでは 8。4 にすると `DEBUG_CH_FIRST` から 4ch） |
| `DEBUG_RATE_HZ` | 4ch: 1000 / 8ch: 500 | UART DBG の出力レート。ES7210 は `SAMPLE_RATE_HZ`（8kHz 以上）で動かし、`SAMPLE_RATE_HZ / DEBUG_RATE_HZ` 個ごとに 1 行出す。通信速度に収まらない設定はビルドエラーになる |
| `DEBUG_AVERAGE` | 1 | 1: 間引く区間の平均／0: N 個ごとに 1 サンプル／2: 区間のピーク（絶対値の最大。マイクが反応しているかを見るのに便利） |
| `DEBUG_CH_FIRST` | 0 | `DEBUG_CH` を 4 にしたときの先頭（0: CH1-4、4: CH5-8） |
| `OUT_MODE_DEBUG_UART_PEAK` | 未定義 | 定義すると UART DBG は、`DEBUG_PEAK_MS`（500ms）ごとに各チャンネルのその区間の振幅（最大値 − 最小値）を 1 行で出す（`CH1pp,CH2pp,...`）。`DEBUG_RATE_HZ` と `DEBUG_AVERAGE` は使わない |
| `UART_G43` | 未定義 | 未定義: UART DBG を USB シリアル（ログと同じポート）へ出す／定義: ハードウェア UART（TX ピンは `PIN_DEBUG_UART_TX`：S3 は G43、P4 は G37）へ出す |
| `DEBUG_UART_BAUD` | 921600 | `UART_G43` のときの UART DBG の通信速度 |
| `SINGLE_ES7210` | 未定義 | 定義すると ES7210 A だけを使う 4ch のテスト用ビルド（I2S1・ES7210 B は使わない） |

データレートは `Fs × 8ch × バイト数` です（32kHz/16bit で 512 kB/s、48kHz/16bit で 768 kB/s、48kHz/24bit で 1152 kB/s）。

**USB Audio の上限**
- **Stamp-P4（UAC2 / High-Speed）**: 8ch・48kHz・32bit まで使えます。パケット間隔（1ms / 500µs / 250µs）は、1パケットが 1024 バイトに収まる最長の間隔が自動で選ばれます（例: 48kHz/16bit → 1ms で 784B、48kHz/32bit → 500µs で 800B）。
- **StampS3（UAC1 / Full-Speed）**: ESP32-S3 の USB FIFO は合計 1KB しかないため、1ms あたりのパケットは 624 バイト以内に収める必要があります。
  - 16bit: 38kHz まで（**32kHz 推奨**）
  - 24bit: 25kHz まで（24kHz）
  - 32bit: 18kHz まで（16kHz）

  この範囲を超える設定では USB Audio は自動で無効になります（SD と SERIAL は使えます）。

## 3. 操作

**USB シリアルのコマンド（両ボード共通）**

1文字のコマンドを送ります（改行は不要）。Stamp-P4 では、基板の USB-C（USB Serial/JTAG）と USB HS 側の CDC のどちらからでも受け付けます。ログも両方に出ます。

| コマンド | 動作 |
|---|---|
| `r` | SD モードに切り替えて録音開始 |
| `s` | SERIAL モードに切り替えて送信開始 |
| `u` | USB AUDIO モードに切り替え（録音アプリが入力を開くとデータが流れる） |
| `d` | UART DBG モードに切り替えて出力開始 |
| `x` | 停止 |
| `i` | 情報を JSON で返す（停止中のみ） |
| `l` | モード・状態と、各チャンネルのピークレベル（dBFS）を1行で返す。`drop`: リングバッファがあふれて捨てたフレーム数、`ur`: USB Audio で送るデータが足りなかったパケット数、`pk`: USB Audio で送ったパケット数（1 秒に約 1000 増えれば正常）、`pf`: 送信フレームを合わせ直した回数、`ab`: PC が録音を止めたときに取り消したパケット数 |
| `R` | ES7210 A/B のレジスタ REG00〜4F を16進で表示し、MA8 が書き込んだ値と照合する（違う値には `*` を付け、最後に `all written registers ok` または `MISMATCH` を表示）。SERIAL で送信している間は無視する |
| `C` | 前回の起動がどう終わったか（リセット要因）を表示する。クラッシュで再起動した場合は、フラッシュに残ったコアダンプから、落ちたタスク名・PC・バックトレースも表示する（StampS3。起動ログにも出る）。割り込みウォッチドッグなどコアダンプが残らない場合は、G43（UART0、115200 baud）に出るパニック出力を見る。SERIAL で送信している間は無視する |
| `g24` + 改行 | ゲインを 24dB に設定 |

**StampS3 のボタン／G0 スイッチ**: 本体ボタン（M5Unified の BtnA。M5Capsule では G42、StampS3 単体では G0）を押すか、G0 を GND に落としたときの動作です。
- **長押し（`LONG_PRESS_MS` = 1 秒）**：**SD 記録モードと USB AUDIO モードを切り替え** ます。1 秒たった時点で、離す前に切り替わります。SD 録音中なら、録音を止めて保存してから切り替えます。SD／USB AUDIO 以外のモードから長押しすると、SD モードになります。
- **短押し**（離したときに動作）は、モードによって次のように動きます。
- **OUT_MODE_SD のとき**：**SD への WAV 録音が開始／停止** します。USB シリアルに開始・停止のメッセージが出ます（`SD rec: start -> /MA8/R0001_00.WAV ...`、`SD rec: stop, 12.3 s, 1 files, dropped 0 frames`）。マイクの振幅は出しません。SD のマウントやファイル作成に失敗すると、`SD mount failed` などを表示します。
- **USB AUDIO モードのとき**：何もしません（誤ってモードを抜けないため）。
- **それ以外のモードのとき**：**UART DBG の出力が開始／停止** します。停止中に押すと、UART DBG モードに切り替えて出力を始めます。SERIAL で送信している間は、誤って止めないように無視します。
- 2つは1つのボタンとして扱います（30ms のチャタリング除去）。StampS3 単体では本体ボタンも G0 なので、1回押しても2回切り替わることはありません。
- SERIAL／UART DBG モードは、USB シリアルのコマンドでも選べます。
- G0 は内部プルアップです。BOOT ストラップでもあるので、**押したままリセットや電源投入をすると書き込みモードで起動します。**

LED（本体の WS2812、FastLED で点灯）: **黄 = I2C（ES7210）を初期化中**、**緑 = SD 記録モード**（UART DBG／SERIAL モードも緑）、緑の点滅 = SD 録音中（または UART DBG／SERIAL の出力中）、**青 = USB AUDIO モード**、青の点滅 = PC が USB AUDIO を録音中、赤 = I2S の初期化に失敗

**I2C の初期化**: 起動時に ES7210 の設定手順を 1 ステップずつ USB シリアルへログ出力します（probe、reset、HPF、clock、TDM format … と、設定したレジスタを読み戻して確認する verify）。応答がないときはバス上のアドレスを一覧で出します。失敗したら `I2C_RETRY_MS`（1 秒）ごとに I2C バスを開き直し、成功するまで繰り返します（その間 LED は黄のまま）。

**SERIAL モード（PC側）**

```
pip install pyserial numpy
python tools/ma8_serial_rx.py COM5 -t 10 -o rec.wav
```

Stamp-P4 では、USB HS 側の CDC の COM ポートを指定してください（基板の USB-C 側ではありません）。

**UART DBG モード**: ES7210 の動作確認用です。出力先は次のとおりです。
- 既定（`UART_G43` 未定義）: 初期化ログと同じ USB シリアルに出します（Stamp-P4 では HS 側 CDC と基板の USB-C の両方）。`pio device monitor` でそのまま見られます。ログの行が混ざることがあります。
- `UART_G43` を定義したとき: USB-UART 変換器の RX を TX ピン（S3: G43、P4: G37）につなぎ（GND も共通に）、921600bps で開きます。

開始すると、最初にラベル行（`CH1,CH2,...,CH8`、4ch ビルドでは `CH1,...,CH4`）を出し、その後は 1 行ごとに全チャンネルの値（SAMPLE_BITS 単位の整数）を出します。8ch で見るときは config.h の `#define SINGLE_ES7210` をコメントアウトしてください。Arduino IDE のシリアルプロッタなどでそのまま波形を見られます。ES7210 は 8kHz 未満では動かないため、サンプリングは `SAMPLE_RATE_HZ` のまま行い、出力だけを間引いています。

**USB AUDIO モード**: 「MA8 8ch Microphone」（`SINGLE_ES7210` のときは 4ch）として認識されます。録音アプリが入力を開いている間だけデータを流します。

## 4. 同期の確認

起動ログ（シリアルモニタ）に次の行が出ます。

```
BCLK stops while I2S0 stopped: yes (A/B start alignment guaranteed)
```

`no` と表示された場合は、A/B の取り込み開始が1フレームずれている可能性があります。その場合は、A の MIC1 と B の MIC1 に同じ音を入れて、次のコマンドで確認してください。

```
python tools/ma8_serial_rx.py COM5 -t 3 --sync-check
```

`CH5 delay vs CH1 = 0 samples` と表示されれば揃っています。

## 5. ファイル構成

```
platformio.ini            StampS3 のビルド設定（espressif32@6.13.0）
p4/platformio.ini         Stamp-P4 のビルド設定（pioarduino、core_dir 分離、src_dir = ../src）
p4/variants/m5stack_stamp_p4/pins_arduino.h   Stamp-P4 用 Arduino バリアント
include/config.h          設定（#define）、ボードごとのピン割り当て
src/main.cpp              UI、コマンド、モード管理、SD/シリアル/デバッグ UART 出力（共通）
src/capture.h             I2S0/I2S1 TDM 受信 → 8ch フレーム化 → リングバッファ
src/s3/capture.cpp          StampS3 実装（legacy driver/i2s.h）
src/p4/capture.cpp          Stamp-P4 実装（driver/i2s_tdm.h）
src/usb_audio.h           USB Audio マイク（TinyUSB アプリケーションクラスドライバ）
src/s3/usb_audio.cpp        StampS3 実装（UAC1、Full-Speed）
src/p4/usb_audio.cpp        Stamp-P4 実装（UAC2、High-Speed）
src/es7210.*              ES7210 レジスタ設定（I2C は S3: M5.Ex_I2C、P4: Wire）
src/serial_stream.*       バイナリパケット送信
src/wav_writer.*          WAV 書き込み
src/ringbuf.h             SPSC リングバッファ
tools/ma8_serial_rx.py    PC 受信スクリプト
```
