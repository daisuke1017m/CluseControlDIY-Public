#pragma once

#include <Arduino.h>
#include "Config.h"

/**
 * PedalIO クラス
 *  - メインアクセルペダル (GPIO 34) とサブアクセルペダル (GPIO 35) の電圧を読み取り、0~100% の割合で返す
 *  - ブレーキスイッチ (GPIO 33) の状態を取得
 *  - PWM 出力 (GPIO 26, 27) でアクセル制御信号を出力 (0~100% → 0~255)
 */
class PedalIO {
public:
    PedalIO();
    void init();

    // 0~100% のペダル開度を取得
    float readMainPedal() const;
    float readSubPedal() const;

    // 実際のペダル入力電圧を取得
    float readMainPedalVoltage() const;
    float readSubPedalVoltage() const;

    // PWM 出力: percent (0~100)
    void setMainPWM(float percent);
    void setSubPWM(float percent);

private:
    // 補助関数: float型の線形近似
    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
    // ルックアップテーブルを用いた非線形キャリブレーション補正
    static float getCalibratedVoltage(float target_v);
};
