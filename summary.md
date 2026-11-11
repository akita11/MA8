# MA8 引き継ぎメモ（VSCode の Claude 向け）

作成日: 2026-10-20 / 更新: 2026-11-04（UART DBG の 8ch 出力、PC 移行）

> 別の PC で作業を続けるときは、このファイルと README.md を読めば前提がわかるようにしてある。Claude Code のチャット履歴（`~/.claude/projects/.../*.jsonl`）は個人情報を含むので、公開リポジトリには置いていない。

## 0. 直近の作業（2026-11-04）

- UART DBG（`OUT_MODE_DEBUG_UART`）を全チャンネル出力にした。`DEBUG_CH`（既定 `NUM_CH`）、8ch では `DEBUG_RATE_HZ` 500（4ch は 1000）。
  - 通信速度（921600bps）に収まらない組み合わせは `static_assert` でビルドエラーにしている。
  - `DEBUG_AVERAGE 2` でピーク（|x| の最大）を出す。平均だと音声の交流成分が消えるので、マイクが反応しているかを見るならピークがよい。
  - StampS3 の 8ch（16/32bit）でビルドが通ることを確認済み。実機では未確認。
- 8ch で使うには config.h の `#define SINGLE_ES7210` をコメントアウトする（現在は定義したまま）。
- 次にやること: 8ch の UART DBG で MIC1〜4（ES7210 B も）を1本ずつつなぎ、チャンネル対応表を取る（§2 の未解決事項）。

## 1. 目的

- ES7210（4ch ADC）×2 で 8ch 同時サンプリングを行う装置を作る。
- 出力は4モード：SD（WAV）、USB シリアル（バイナリ）、USB オーディオ（UAC）、デバッグ UART（4ch は約 1kHz、8ch は約 500Hz の CSV）。
- PlatformIO を使う。サンプリングレートとビット数は `#define`（`include/config.h`）で設定する。
- 対象ボード：
  - **M5Stack Stamp-P4（ESP32-P4）＝本命**。USB HS + UAC2 で 8ch/48kHz/24bit を USB Audio で出すため。
  - **M5StampS3（M5Capsule 内）**＝既存。USB FS の制約で USB Audio は 16bit/38kHz まで。
  - CoreS3 は削除済み。
- 旧版 `../MA8_old` は CM2248（AD7606 互換）を使った別設計。コードのスタイル（config.h、ringbuf、wav_writer、README の書き方）を踏襲している。

## 2. 現状

| 項目 | 状態 |
|---|---|
| 接続設計（ピン、回路方針） | 完了（README.md §1） |
| StampS3 ファームウェア | 実機で ES7210 A の 4ch 取得（SINGLE_ES7210、UART DBG）まで動作。クロック 8.192MHz / 2.048MHz / 32kHz を確認済み |
| Stamp-P4 ファームウェア | 作成済み。**実機では未テスト**。PlatformIO でビルド通過（4ch/8ch、16/24/32bit、48kHz、rev<3 / rev3） |
| PC 受信スクリプト | `tools/ma8_serial_rx.py`。合成データでパーサと同期チェックを確認済み。4ch パケットにも対応 |

**チャンネル対応（2026-11 にソフトで対応済み）**: 8ch で 1 本ずつ測った結果、1 チップ内の受信スロット順は slot0=MIC2, slot1=MIC4, slot2=MIC1, slot3=MIC3（MIC3 は推定）だった。config.h の `ES7210_SLOT_OF_MIC {2,0,3,1}` で並べ替え、CH1..4 = A MIC1..4、CH5..8 = B MIC1..4 にしている（A/B が入れ替わって見えたのは DIN_A/DIN_B の配線ミス）。半フレームずれ（LRCK 極性）が原因なら、MIC1/3 と MIC2/4 の間に 1 サンプルの時間ずれが残る可能性がある。
→ `ES7210_LRCK_INVERT`（既定 1）を追加した。ESP の GPIO マトリクスでパッドへの LRCK 出力を反転し、8ch では I2S1 の入力側も反転し戻す（ES7210 から見た LRCK だけが反転する）。反転時の表は `{0,2,1,3}`。**実機で CH1..4 = MIC1..4 になることを確認済みで、この仕様（LRCK 反転 + `{0,2,1,3}`）で確定**（2026-11-10）。以下は当初のメモ。

（旧）**未解決: チャンネル対応**。StampS3 で MIC1 だけつなぐと CH3 が振れる（CH1/2/4 は 0）。MIC2〜4 も順につないで 4×4 の対応表を取る必要がある。2スロット回転（MIC1→CH3, MIC2→CH4, MIC3→CH1, MIC4→CH2）なら LRCK のフレーム先頭が半フレームずれている（CH1/2 と CH3/4 が別フレーム）ので、ES7210 REG11 の LRCK 極性か ESP 側の WS 極性で直す。それ以外の並びなら、ES7210 自体のスロット順としてソフトで並べ替える。

## 3. ビルド環境（重要）

- **StampS3**: ルートの `platformio.ini`、`platform = espressif32@6.13.0`（Arduino 2.0.17 / IDF 4.4）、通常の `~/.platformio`。
- **Stamp-P4**: `p4/platformio.ini`（別プロジェクト、`src_dir = ../src`）、pioarduino `55.03.312-1`（Arduino 3.3.12 / IDF 5.5.5）。
  - `core_dir = ~/.platformio-pioarduino` で隔離している。pioarduino は core の penv に自前の PlatformIO フォーク（`pioarduino` パッケージ）を上書きインストールし、`toolchain-*` などの `@` 付きディレクトリを削除するため、通常の `~/.platformio` で使うと他プロジェクトを壊す（実際に一度壊して復旧した）。
  - pioarduino はコマンドを PowerShell / cmd から実行すること。Git Bash（MSYS）からだと `idf_tools.py` が「MSys/Mingw is not supported」で失敗する。
  - 公式の Stamp-P4 手順（pioarduino 54.03.21）は esptool 5.0.0-dev1 が click 8.3 と非互換でビルドできない。
- 新しい PC では、PlatformIO（VSCode 拡張）を入れて `pio run -e m5stamps3` を実行すれば espressif32@6.13.0 が入る。`pio run -d p4` を初めて実行すると、pioarduino 一式が `~/.platformio-pioarduino` に入る（PowerShell から実行すること）。
- ソース切り替え: `build_src_filter` で `src/s3/` と `src/p4/` を切り替える。共通は main.cpp、es7210、serial_stream、wav_writer、ringbuf。

## 4. 設計の要点（理由つき）

### I2S 構成（両ボード共通の考え方）
- ESP32 の I2S は TDM 時にデータ入力が 1 本しかない。そのため 2 ポート構成にしている：
  - **I2S0 = マスター**：MCLK/BCLK/LRCK を出力し、ES7210 A のデータを受信
  - **I2S1 = スレーブ**：ES7210 B のデータを受信
- I2S1 の BCLK/LRCK は、同じパッドの入力を有効にしたうえで、I2S0 の出力信号と I2S1 の入力信号を GPIO マトリクスで同じパッドにつないでいる（`installI2S()`）。P4 は `i2s_periph_signal[0].m_rx_bck_sig` / `[1].s_rx_bck_sig` と `gpio_input_enable()` を使う。
- **A/B のフレーム整列**（`syncRestart()`）：両ポート停止 → I2S1 → I2S0 の順に開始。S3 は legacy ドライバの DMA キューを `i2s_read(timeout 0)` で空にしてから開始。P4 は `i2s_channel_enable()` が DMA/FIFO をリセットするので不要。
- この整列は「I2S0 停止中は BCLK も止まる」ことが前提。起動時に BCLK のトグルを読んで検査し、`status().bclk_stops` に入れている（8ch 時のみ）。
- **P4 の MCLK**：IDF 5 の TDM ドライバは、複数スロット受信で MCLK/BCLK が 2 以下になると MCLK 倍率を自動で引き上げる（記憶ベース、ドライバのソースは未確認。ヘッダにも 24bit は 384 にせよという注記がある）。ES7210 側の設定とずれないよう、スロット 32bit（24/32bit）のときは MCLK=512Fs にしている。実機で MCLK を測って確認すること。ES7210 側は adc_div=2、LRCK 分周 512（`es7210.begin(..., mclk_multiple)`）。
- ES7210 のデータシートには TDM カスケード（SDOUT2/TDMIN で最大16ch）の記載があるが、レジスタの詳細が手元になかったので採用していない。

### ES7210 設定（`es7210.cpp`）
- esp-codec-dev / ESP-ADF の es7210 ドライバのシーケンスを参考にした：スレーブ、TDM（REG12=0x02）、I2S フォーマット、ワード長（16bit は 0x60、24/32bit は 32bit ワード 0x80）、REG02=0xC0|adc_div、REG07=0x20、LRCK 分周 = 256×adc_div。
- 単速（8〜48kHz）のみ対応。
- I2C：S3 は `M5.Ex_I2C`、P4 は Arduino `Wire`（`USE_M5UNIFIED` で切り替え）。アドレス A=0x40、B=0x41。

### データの流れ
- capture タスク（core0、高優先度）：I2S0 と I2S1 から 160 フレームずつ読み、インターリーブして（CH1-4=A、CH5-8=B）SAMPLE_BITS に変換し、`FrameRing` に push する。
- リングバッファ：Stamp-P4 は PSRAM 8MB、StampS3 は内部 SRAM 128KB（32kHz/16bit で約 0.25 秒分）。
- リングの消費者は常に1つだけ（SD / SERIAL / UART DBG：loop タスク、USB AUDIO：TinyUSB タスク）。切り替えは `usb_audio::setActive(false)` → 5ms 待つ → `discard`（`main.cpp: setMode()`）。

### USB
- 両ボードとも TinyUSB（`ARDUINO_USB_MODE=0`、`CDC_ON_BOOT=1`）。UAC は `tinyusb_enable_interface(USB_INTERFACE_CUSTOM, …)` をグローバルコンストラクタで登録し、`usbd_app_driver_get_cb` でクラスドライバを提供する。
- **Stamp-P4（`src/p4/usb_audio.cpp`、UAC2 HS）**
  - Arduino 3.3.12 の TinyUSB 0.21 は P4 で OTG HS（UTMI、rhport 1）を使う。内蔵の UAC クラスは `CFG_TUD_AUDIO_MAX_N_CHANNELS 2` でビルド済みなので使えず、自前のクラスドライバにしている。
  - 構成：IAD + AC（Clock Source 内部固定 → IT マイク → OT）+ AS alt0/alt1、iso IN 非同期。Clock Source の SAM_FREQ（CUR/RANGE）と CLOCK_VALID に応答、SET_CUR は受けて無視。
  - iso EP は `usbd_edpt_iso_alloc()`（open 時）と `usbd_edpt_iso_activate()` / `usbd_edpt_close()`（SET_INTERFACE 時）。S3 のような FIFO やパリティの手当ては不要（dwc2 ドライバが処理）。
  - パケット間隔は 1パケット ≤1024B に収まる最長を config.h で自動選択（`UAC_BINTERVAL`）。
  - 基板の USB-C は USB Serial/JTAG（FS）。HS の D+/D- は底面パッド（USB2_HOST_DP/DM）にしか出ていないので、外付けコネクタが必要。
  - ログとコマンドは HS 側 CDC（`Serial`）と USB-C（`HWCDC JtagSerial`、Arduino は USB_MODE=0 だと `HWCDCSerial` を定義しないので自前で作成）の両方。
  - SERIAL モードは `Serial.write()`（`SERIAL_USE_RAW_EP 0`）。
- **StampS3（`src/s3/usb_audio.cpp`、UAC1 FS）**
  - `dcd_esp32sx` は IN FIFO を均等割り（1つ約 188B）するので `fixFifo()` で DIEPTXF を書き換え、約 624B を音声用にしている（`UAC_MAX_PKT_BYTES ≤ 624`、16bit は 38kHz まで）。
  - iso EP は SET_CONFIG で 1 回だけ開く（dcd が FIFO を解放しないため）。
  - 旧 dcd は incomplete iso を処理しないので、`watchdogTask` が DIEPCTL の even/odd フレームビットを打ち直す。
  - SERIAL は CDC の bulk IN（0x84）へ `usbd_edpt_xfer` で直接送る（ダブルバッファ）。
- パケット長の調整（両ボード）：リングの水位（IIR 平均）を見て ±1 フレームで調整。目標水位は 2×I2S ブロック + 2ms 分。
- CRC は `esp_rom_crc32_le(0, …)`。zlib.crc32 と一致する想定だが**未確認**。

### SD
- Stamp-P4：SDMMC スロット1（GPIO マトリクス）4bit、20MHz。Arduino の esp32p4 バリアントは Function EV ボード用（スロット0固定、LDO 給電、GPIO45 電源制御）なので、`p4/variants/m5stack_stamp_p4/` に SDMMC 定義を外したバリアントを置いている。
- StampS3：M5Capsule 内蔵スロット（SPI）。

## 5. 実機立ち上げチェックリスト（Stamp-P4、未実施）

1. 基板の USB-C から書き込み、起動ログ（`pio device monitor -d p4`）で次を確認：
   - `ES7210 A(0x40): OK`（8ch なら B も）、`BCLK stops while I2S0 stopped: yes`、`MCLK = 256 x Fs`
2. オシロで MCLK（256Fs、32bit スロットなら 512Fs）、BCLK（16bit なら 64Fs、32bit なら 128Fs）、LRCK（Fs、デューティ 50%）。
3. `l` コマンドでレベルを見ながら各マイクを叩き、チャンネル対応を確認（§2 の未解決事項）。
4. `d` で UART DBG（G37）が出るか。
5. SD：`r` で 10 分録音し、`drop:0` のままか、ファイル分割と WAV ヘッダが正しいか。
6. USB HS をつなぎ、デバイスマネージャ／USBTreeView で High-Speed の複合デバイス（CDC + UAC2）として認識されるか。
7. USB AUDIO：`u` → Windows のサウンド設定で「MA8 8ch Microphone」が見えるか、Audacity（WASAPI）で 8ch 録音できるか、長時間で `ur` が増えないか。
8. SERIAL：HS 側 CDC の COM で `python tools/ma8_serial_rx.py COMx -t 30`、lost=0 / crc_err=0、48kHz/24bit（1.15MB/s）が出るか。
9. 同期：A と B の MIC1 に同じ音を入れて `--sync-check` で 0 サンプル。

## 6. 既知の制約と TODO 候補

- StampS3 はリングが小さい（約 0.25 秒）ので、SD の書き込み遅延でドロップする可能性がある。
- ES7210 のデジタルボリューム、HPF の on/off、ch ごとのゲインは未実装。倍速モード（64〜96kHz）も未実装。
- Stamp-P4 にはボタン・LED がないので、操作はシリアルコマンドのみ。
- `CORE_DEBUG_LEVEL` が board 定義と二重定義になり、warning が出ることがある（無害）。

## 7. ピン（再掲）

| 信号 | Stamp-P4（2.54mm DIP ピン） | StampS3（M5Capsule、2.54mm ピン） |
|---|---|---|
| MCLK | G0 (LPG0) | G1 |
| BCLK | G35（BOOT ストラップ、10k プルアップ） | G5 |
| LRCK | G38（U0RXD） | G7 |
| DIN_A（ES7210 A SDOUT1） | G17 | G9 |
| DIN_B（ES7210 B SDOUT1） | G19 | G44 |
| SDA / SCL | G26 / G27（USB FS PHY パッド） | G13 / G15 |
| UART DBG TX | G37（U0TXD） | G43 |
| SD | SDMMC 4bit：CLK G12, CMD G10, D0 G8, D1 G1, D2 G21, D3 G23 | M5Capsule 内蔵（CS=G11, SCK=G14, MOSI=G12, MISO=G39） |
| USB（Audio + CDC） | USB2_HOST_DP / DM（底面パッド、DIP 外） | 基板の USB-C |

## 8. 参考

- ES7210 データシート: https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/6/7563.ES7210.pdf
- esp-codec-dev の es7210 ドライバ（レジスタシーケンスの元）: https://components.espressif.com/components/espressif/es7210
- Stamp-P4 ドキュメント / 回路図 / ピンマップ: https://docs.m5stack.com/en/core/Stamp-P4
- TinyUSB 0.16.0 `dcd_esp32sx.c`（S3 の FIFO 割当とパリティ処理の根拠）、TinyUSB 0.21 `usbd_pvt.h`（P4 の iso API）
- ESP-IDF v4.4.7 `components/driver/i2s.c`（S3 legacy I2S）、ESP-IDF v5.5 `driver/i2s_tdm.h`（P4）
