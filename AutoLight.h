#pragma once
#include <Arduino.h>

// =============================================================
// AutoLight パラメータ
// =============================================================

// ─── ピン設定 ─────────────────────────────────────────────
#define AUTOLIGHT_SENSOR_PIN    36      // ADC VP: 光抵抗センサ (GPIO36専用で ADC_11db を設定)
#define AUTOLIGHT_OUTPUT_PIN    15      // オートライト出力 (Active High: HIGH=点灯)
// ※SWは GPIO15 出力線上に物理スイッチとして配置。GPIO入力によるSW読み取りなし。

// ─── ADC 設定 ─────────────────────────────────────────────
#define AL_MAX_ADC_VALUE        4095.0f
// analogSetPinAttenuation(AUTOLIGHT_SENSOR_PIN, ADC_11db) を AutoLight::init() で個別設定。
// ペダルADC (GPIO34/35) は PedalIO::init() の ADC_6db 設定を維持する。

// ─── 明るさ閾値 [%] ──────────────────────────────────────
#define AL_THRESHOLD_TUNNEL     20.0f   // トンネル判定閾値（即点灯）
#define AL_THRESHOLD_DUSK       45.0f   // 夕暮れ判定閾値（長距離点灯）
#define AL_THRESHOLD_DUSK_INIT  30.0f   // 起動時即点灯閾値
#define AL_THRESHOLD_OFF        60.0f   // 消灯判定閾値（通常）
#define AL_THRESHOLD_OFF_TUNNEL 54.0f   // 消灯判定閾値（トンネル出口）

// ─── 累積距離閾値 [m] ──────────────────────────────────────
#define AL_DIST_TUNNEL          20.0f   // トンネル点灯トリガー距離
#define AL_DIST_DUSK            500.0f  // 夕暮れ点灯トリガー距離
#define AL_DIST_OFF             30.0f   // 通常消灯トリガー距離
#define AL_DIST_OFF_TUNNEL      100.0f  // トンネル出口消灯トリガー距離

// ─── 時間・車速パラメータ ──────────────────────────────────
#define AL_TIME_LOW_SPEED_TRIGGER  5000 // 低速時点灯トリガー時間 [ms]
#define AL_LOW_SPEED_KPH           5.0f // 低速判定車速 [km/h]
#define AL_STARTUP_DELAY_MS        500  // 起動後の即時判定開始ディレイ [ms]
#define AL_MIN_ON_TIME_MS          5000 // 最小点灯継続時間 [ms]

/**
 * AutoLight クラス
 *
 * CAN車速から計算した走行距離と光センサの値を用いて、
 * オートライトのON/OFFを制御する。
 *
 * 出力: GPIO AUTOLIGHT_OUTPUT_PIN (Active Low: LOW=点灯)
 * SW:   物理スイッチを出力線上に配置。GPIO入力によるSW読み取りなし。
 *
 * 光センサ: GPIO AUTOLIGHT_SENSOR_PIN (GPIO36/VP)
 *   analogSetPinAttenuation(AUTOLIGHT_SENSOR_PIN, ADC_11db) を init() で設定。
 *   ペダルADC(GPIO34/35)への影響なし。
 */
class AutoLight {
public:
    AutoLight();

    // 初期化 (setup() 内で呼ぶ)
    // GPIO設定・analogSetPinAttenuation・起動ディレイ用タイムスタンプを初期化する
    void init();

    // 毎制御周期で呼ぶ
    //   speedKmh : CAN車速 [km/h]
    //   canActive: CAN通信が正常に行われているか
    //   dt       : 前回呼び出しからの経過時間 [秒]
    void update(float speedKmh, bool canActive, float dt);

    // GPIO15 の現在出力状態 (true = 点灯中 = GPIO LOW)
    bool isLightOn() const;

    // ─── OLED 表示用ゲッター ────────────────────────────────────
    // 正規化輝度 [0〜100 %]  (最重要表示)
    float getBrightness() const;

    // DD: 夕暮れ(薄暗い)蓄積距離 [m]
    float getAccumDistMid() const;

    // BD: 明るい→消灯方向の蓄積距離 [m]
    float getAccumDistOff() const;

    // 点灯ロジック状態 (SWとは無関係なロジック判定結果)
    bool getLogicLightOn() const;

private:
    bool     _startupChecked;
    bool     _logicLightOn;
    uint32_t _lightOnTimestamp;
    
    bool     _canStarted;         // CAN通信が開始されたか
    uint32_t _canStartTimestamp;  // CAN通信が開始された時刻

    float    _accumDistLow;       // ND用: トンネル蓄積 (内部ロジックのみ、OLED表示なし)
    float    _accumDistMid;       // DD: 夕暮れ蓄積
    float    _accumDistOff;       // BD: 消灯方向蓄積
    float    _accumDistOffTunnel; // 内部ロジックのみ

    uint32_t _lowBrightnessStart;
    float    _normalizedBrightness;
};
