
[readme.md](../readme.md)

# ESP32ピン配置
| 機能                       | ピン番号    | 備考                             | 種類   |
| :------------------------- | :---------- | :------------------------------- | :----- |
| アクセル Main              | GPIO 34     | ADC1                             | Input  |
| アクセル Sub(予備)         | GPIO 35     | ADC1                             | Input  |
| フォトレジスター           | GPIO 36(VP) | ADC1                             | Input  |
| 車速パルス                 | GPIO 32     | パルス                           | Input  |
| OLED表示切替スイッチ       | GPIO 33     | 内部プルアップ                   | Input  |
| クルーズ操作SW（抵抗分圧） | GPIO 14     | 外部抵抗分割（Cancel/Res/Set）   | Input  |
| オートライトスイッチ       | GPIO 15     | リレーモジュール制御 (ActiveLow) | Output |
| ESP32Ready Out             | GPIO 12     | 制御開始後にHigh (基板LED連動)   | Output |
| I2C (SCL)                  | GPIO 22     | OLED                             | 通信   |
| I2C (SDA)                  | GPIO 21     | OLED                             | 通信   |
| CAN (TX)                   | GPIO 17     |                                  | 通信   |
| CAN (RX)                   | GPIO 16     |                                  | 通信   |
| アクセルPWM Main           | GPIO 26     |                                  | Output |
| アクセルPWM Sub(予備)      | GPIO 27     |                                  | Output |
| SPI (SCK)                  | GPIO 18     | ディスプレイ共通                 | 通信   |
| SPI (MOSI)                 | GPIO 23     | ディスプレイ共通                 | 通信   |
| SPI (MISO)                 | GPIO 19     | ディスプレイ共通                 | 通信   |
| SPI (CS 1)                 | GPIO 13     | ディスプレイ1用                  | 通信   |
| SPI (CS 2)                 | GPIO 25     | ディスプレイ2用                  | 通信   |
| SPI (CS SDカード)          | GPIO 5      |                                  | 通信   |
