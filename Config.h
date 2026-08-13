#pragma once
#include <Arduino.h>

// Serial communication
#define SERIAL_BAUDRATE 921600

// CAN (TWAI) pins
#define ESP32_CAN_TX_PIN GPIO_NUM_17
#define ESP32_CAN_RX_PIN GPIO_NUM_16

// 車輪速パルスピン（トランジスタ反転回路経由、RISINGエッジで検出）
#define VEHICLE_SPEED_PULSE_PIN 32

// Pedal and brake pins
#define MAIN_PEDAL_PIN 34
#define SUB_PEDAL_PIN 35

// PWM pins
#define PWM_MAIN_PIN 26
#define PWM_SUB_PIN 27

// HMI pins
#define OLED_MODE_SWITCH_PIN 33 // OLED画面表示切替スイッチ
#define CRUISE_SW_PIN 14        // クルーズコントロール操作スイッチ（ADC抵抗分圧）



// Cruise control settings
#define MIN_TARGET_SPEED 10.0f

// Safety / Kill switch
#define KILL_SWITCH_PIN 12
#define KILL_SWITCH_DELAY_MS 500

// PWM settings
#define PWM_MAIN_CHANNEL 0
#define PWM_SUB_CHANNEL 1
#define PWM_FREQ 1000 // 1kHz (実車合わせ)
#define PWM_RESOLUTION 12 // 12bit (実車合わせ)

// =============================================================
// キャリブレーションパラメータ（実測データに基づく境界値）
// =============================================================
// 【入力側】シリアルウィンドウで観測された電圧値（2.2V基準）
//  ESP32のADCは、内部の基準電圧（約1.1V）を超える電圧をそのまま測定することができません。
//  ADC_6db 約 0V 〜 2.2V まで ADC_11db 約 0V 〜 3.3V まで
//  ペダル電圧0.6～3.2V。5Vで吊られて、3.3Vぎりぎりなので分圧して約半分をインプットとしている
#define IN_MIN_V 0.292f
#define IN_MAX_V 1.838f

// 【出力側】オペアンプの先から「実際の入力電圧」を出すための目標値
// 仕様決定: 実車のアクセルペダル目標範囲は 0.68V 〜 3.2V
// ※線形近似からルックアップテーブル方式による非線形キャリブレーションに移行したため、
//   OUT_MIN_DUTY/OUT_MAX_DUTY は廃止されました。
// ※PWM出力計算のため、ESP32のピン出力電圧（基準）を定義します。
#define SYSTEM_VCC 3.3f

// Control cycle
#define CONTROL_CYCLE_MS 10

// OLED設定 (SSD1306 128x64 I2C)
#define OLED_SCREEN_WIDTH  128
#define OLED_SCREEN_HEIGHT  64
#define OLED_I2C_ADDRESS   0x3C

