#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"

/**
 * OledDisplay クラス
 *
 * SSD1306 128x64 OLED にクルーズコントロール + AutoLight の状態を表示する。
 * setRotation(2) (180度反転) で使用するため、物理画面の上下はソフトウェア座標と逆になる。
 *
 * 画面レイアウト (ソフトウェア座標 / setRotation(2) 適用で物理上下逆):
 *   Y= 0~15  [青エリア] PWM バーグラフ         → 物理: 上側
 *   Y=16     : 境界線
 *   Y=17~27  [青エリア] BD バーグラフ           → 物理: 上寄り中央
 *   Y=28     : 境界線
 *   Y=29~39  [青エリア] DD バーグラフ           → 物理: 下寄り中央
 *   Y=40     : 境界線
 *   Y=41~52  [青エリア] BR (明るさ) バーグラフ  → 物理: 下寄り中央
 *   Y=53     : 境界線
 *   Y=54~63  [黄色エリア] CC ON/OFF + 車速 + 設定車速 → 物理: 下側
 *
 * 接続ピン:
 *   SCL → GPIO22, SDA → GPIO21 (ESP32デフォルトI2Cピン)
 */
class OledDisplay {
public:
    OledDisplay();

    // 表示モード定義
    enum DisplayMode {
        MODE_CRUISE_MAIN = 0, // モード①: クルーズコントロール専用（デカ文字車速＋シンプル表示）
        MODE_NORMAL_DETAIL,   // モード②: 現状の詳細画面（PWMバー、ロジック名、車速など）
        MODE_AUTOLIGHT_DEBUG, // モード③: AutoLight デバッグ画面（輝度、蓄積距離、点灯状態など）
        MODE_COUNT
    };

    // 初期化（setup() 内で呼ぶ）
    void init();

    // 表示モードのサイクリック切り替え（GPIO33押下時に呼ぶ）
    void nextDisplayMode();

    // 現在の表示モードを取得
    DisplayMode getDisplayMode() const;

    // 毎制御周期の表示更新データのセット（loop() から呼ぶ、非ブロッキング）
    //   ccEnabled   : クルーズコントロール ON/OFF
    //   targetSpeed : 目標車速 [km/h]
    //   currentSpeed: 現在車速 [km/h]
    //   pwmPercent  : PWM出力 [%]（0〜100）
    //   brightness  : AutoLight 正規化輝度 [%]（0〜100）
    //   accumDistMid: AutoLight DD 夕暮れ蓄積距離 [m]
    //   accumDistOff: AutoLight BD 消灯蓄積距離 [m]
    //   lightOn     : AutoLight 点灯ロジック状態
    // 毎制御周期の表示更新データのセット（loop() から呼ぶ、非ブロッキング）
    //   ccEnabled   : クルーズコントロール ON/OFF
    //   targetSpeed : 目標車速 [km/h]
    //   currentSpeed: 現在車速 [km/h]
    //   pwmPercent  : PWM出力 [%]（0〜100）
    //   brightness  : AutoLight 正規化輝度 [%]（0〜100）
    //   accumDistMid: AutoLight DD 夕暮れ蓄積距離 [m]
    //   accumDistOff: AutoLight BD 消灯蓄積距離 [m]
    //   lightOn     : AutoLight 点灯ロジック状態
    //   acceleration: 加速度 [km/h/s]
    void setValues(bool ccEnabled, float targetSpeed, float internalTargetSpeed, float currentSpeed,
                   float pwmPercent,
                   float brightness, float accumDistMid, float accumDistOff, bool lightOn,
                   float acceleration = 0.0f, const char* logicName = "CTRL_OFF");

    // 実際の描画処理（FreeRTOSの別タスクから呼ぶ、ブロッキング）
    void refresh();

    // 5kph変更時のTarget車速反転表示のトリガー
    void triggerTargetSpeedInvert();

private:
    Adafruit_SSD1306 display;

    // 描画データ共有用ミューテックス
    SemaphoreHandle_t mutex;

    // 表示モード
    DisplayMode _displayMode;

    // 反転表示用制御変数
    unsigned long _targetInvertUntil;

    // ─── クルーズコントロール 描画用データ ───────────────────────
    bool  _ccEnabled;
    float _targetSpeed;
    float _internalTargetSpeed;
    float _currentSpeed;
    float _pwmPercent;
    float _acceleration;
    const char* _logicName;


    // ─── AutoLight 描画用データ ──────────────────────────────────
    float _brightness;
    float _accumDistMid;    // DD: 夕暮れ蓄積距離 [m]
    float _accumDistOff;    // BD: 消灯蓄積距離 [m]
    bool  _lightOn;

    // バーグラフ付き1行を描画するヘルパー関数
    void drawProgressBar(const char* label, float val,
                         float minVal, float maxVal, int y,
                         const float* dots, int dotCount);
};
