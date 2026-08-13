#include "PedalIO.h"
#include "Config.h"

// ルックアップテーブル登録用構造体 (オペアンプ後の実測出力電圧 -> それに対応したプログラム目標設定値)
struct CalibrationPoint {
    float actual_out;  // 最終的にオペアンプの先から得たい実測出力電圧 (y)
    float target_v;    // そのためにプログラム側で設定する目標指示電圧 (x)
};

// 実測データに基づいたキャリブレーション用テーブル（昇順）
// ※実車スペックとして、アクセルオン最小出力は0.68V、最大は3.2Vを想定。
// ※このテーブルは { 得たい実測電圧, そのためにプログラムに与える目標電圧 } の順で記述されています。
const int CALIBRATION_POINTS_COUNT = 16;
const CalibrationPoint CALIBRATION_TABLE[CALIBRATION_POINTS_COUNT] = {
    {0.68f, 0.68f}, // 外挿（0.68V出力を得るための推測値）
    {0.75f, 0.75f},
    {0.80f, 0.80f},
    {0.90f, 0.90f},
    {1.00f, 1.00f},
    {1.10f, 1.10f},
    {1.20f, 1.20f},
    {1.30f, 1.30f},
    {1.40f, 1.40f},
    {1.50f, 1.50f},
    {1.70f, 1.80f},
    {2.00f, 2.00f},
    {2.35f, 2.35f},
    {2.70f, 2.70f},
    {3.00f, 3.00f}, 
    {3.20f, 3.20f}
};


PedalIO::PedalIO() {
}

// Initialize ADC, digital input, and PWM channels
void PedalIO::init() {
    // アナログ入力設定（6dBアッテネータ：測定範囲 0V 〜 約2.2V）
    analogReadResolution(12);
    analogSetAttenuation(ADC_6db);

    // Ensure brake switch pin has pull-up resistor (external pull-up present)
    // pinMode(BRAKE_SWITCH_PIN, INPUT);
    
    // HMIピンの初期化は現在 Cluse_Control_Simulation.ino の setup() 内で行われています

    // PWM出力設定（最新Ver 3.x対応: ledcAttachを使用）
    ledcAttach(PWM_MAIN_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWM_SUB_PIN, PWM_FREQ, PWM_RESOLUTION);
}

float PedalIO::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float PedalIO::readMainPedalVoltage() const {
    long sum = 0;
    for (int i = 0; i < 10; i++) sum += analogRead(MAIN_PEDAL_PIN);
    float raw = sum / 10.0f;
    return (raw / 4095.0f) * 2.2f;
}

float PedalIO::readSubPedalVoltage() const {
    long sum = 0;
    for (int i = 0; i < 10; i++) sum += analogRead(SUB_PEDAL_PIN);
    float raw = sum / 10.0f;
    return (raw / 4095.0f) * 2.2f;
}

float PedalIO::readMainPedal() const {
    float voltage = readMainPedalVoltage();
    float percent = mapFloat(voltage, IN_MIN_V, IN_MAX_V, 0.0f, 100.0f);
    return constrain(percent, 0.0f, 100.0f);
}

float PedalIO::readSubPedal() const {
    float voltage = readSubPedalVoltage();
    float percent = mapFloat(voltage, IN_MIN_V, IN_MAX_V, 0.0f, 100.0f);
    return constrain(percent, 0.0f, 100.0f);
}

// 目標出力電圧（target_v）を実測値として含む要素間を探し、対応するプログラム指示値（V）を逆引き計算する関数
float PedalIO::getCalibratedVoltage(float target_v) {
    // 範囲外（下限）の処理 (目標電圧が最小の実測出力 0.68V 以下の場合は、下限のプログラム目標電圧 0.58V を設定)
    if (target_v <= CALIBRATION_TABLE[0].actual_out) {
        return CALIBRATION_TABLE[0].target_v;
    }
    // 範囲外（上限）の処理 (目標電圧が上限の実測出力 3.00V 以上の場合は、上限のプログラム目標電圧 2.90V を設定)
    if (target_v >= CALIBRATION_TABLE[CALIBRATION_POINTS_COUNT - 1].actual_out) {
        return CALIBRATION_TABLE[CALIBRATION_POINTS_COUNT - 1].target_v;
    }

    // actual_out をキーに探索し、target_v を補間計算
    for (int i = 0; i < CALIBRATION_POINTS_COUNT - 1; i++) {
        if (target_v >= CALIBRATION_TABLE[i].actual_out && target_v <= CALIBRATION_TABLE[i+1].actual_out) {
            float y0 = CALIBRATION_TABLE[i].actual_out; // 実測出力 (下限側)
            float y1 = CALIBRATION_TABLE[i+1].actual_out; // 実測出力 (上限側)
            float x0 = CALIBRATION_TABLE[i].target_v;    // プログラム目標値 (下限側)
            float x1 = CALIBRATION_TABLE[i+1].target_v;    // プログラム目標値 (上限側)
            
            // 目標出力 target_v になるようなプログラム目標値 x を線形補間
            return x0 + (x1 - x0) * (target_v - y0) / (y1 - y0);
        }
    }
    return target_v;
}

void PedalIO::setMainPWM(float percent) {
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    // 得たい目標出力電圧 (0.68V 〜 3.2V) にマッピング
    float target_v = mapFloat(percent, 0.0f, 100.0f, 0.68f, 3.2f);
    
    // キャリブレーション補正
    float control_v = getCalibratedVoltage(target_v);
    
    // 指示電圧 (V) をオペアンプの増幅率（2.0倍）で割り、ESP32ピン電圧に換算してから PWM デューティ比 (12bit: 0〜4095, 基準電圧 3.3V) に変換
    float duty = ((control_v / 2.0f) / SYSTEM_VCC) * 4095.0f;
    int outPwm = constrain(static_cast<int>(duty), 0, 4095);
    
    // 【調査特化シリアル出力（プロッタ対応）】
    // float mainVoltageRaw = readMainPedalVoltage(); // GPIO34上の生入力電圧 (分圧後)
    // Serial.print("GPIO34_Raw_V:"); Serial.print(mainVoltageRaw, 3); Serial.print(",");
    // Serial.print("PedalInput_Pcent:"); Serial.print(percent, 1); Serial.print(",");
    // Serial.print("Target_Out_V:"); Serial.print(target_v, 3); Serial.print(",");
    // Serial.print("Ctrl補正_V:"); Serial.print(control_v, 3); Serial.print(",");
    // Serial.print("PinPWM_V:"); Serial.print(control_v / 2.0f, 3); Serial.print(",");
    // Serial.print("Duty:"); Serial.println(outPwm);

    ledcWrite(PWM_MAIN_PIN, outPwm);
}

void PedalIO::setSubPWM(float percent) {
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    // 得たい目標出力電圧 (0.68V 〜 3.2V) にマッピング
    float target_v = mapFloat(percent, 0.0f, 100.0f, 0.68f, 3.2f);
    
    // キャリブレーション補正
    float control_v = getCalibratedVoltage(target_v);
    
    // 指示電圧 (V) をオペアンプの増幅率（2.0倍）で割り、ESP32ピン電圧に換算してから PWM デューティ比 (12bit: 0〜4095, 基準電圧 3.3V) に変換
    float duty = ((control_v / 2.0f) / SYSTEM_VCC) * 4095.0f;
    int outPwm = constrain(static_cast<int>(duty), 0, 4095);
    
    ledcWrite(PWM_SUB_PIN, outPwm);
}
