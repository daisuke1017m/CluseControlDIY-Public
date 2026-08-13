#pragma once

/**
 * PIDController クラス
 * 目標車速と現在の車速から、PID制御によるアクセルペダル開度のシミュレーション値を計算します。
 * ギアごとに異なるパラメータを保持できます。
 */
class PIDController {
public:
    // コンストラクタ
    PIDController();

    // 制御をリセットする（目標車速が変更された時や制御再開時などに呼ぶ）
    void reset(float initialOutput = 0.0f);

    // 周期ごとの計算を行い、シミュレーションペダル開度(0〜100%)を返す
    // currentSpeed: 現在の車速 (km/h)
    // targetSpeed: 目標車速 (km/h)
    // gear: 現在のギア (10, 20, 30, 40, 50, 60 等)
    // dt: 前回からの経過時間 (秒)
    // accel: 現在の加速度 (km/h/s) [オプション: FF補正用]
    float update(float currentSpeed, float targetSpeed, int gear, float dt, float accel = 0.0f);

    // 人間のアクセル操作に合わせて積分項を逆算・同期する（バックキャルキュレーション）
    void syncIntegral(float speed, float target_speed, int gear, float targetOutput);

    // デバッグ・表示用情報の取得
    struct PIDDebugInfo {
        float pTerm;
        float iTerm;
        float dTerm;
        float ffTerm;
        bool inDeadband;
        const char* logicName;
    };
    PIDDebugInfo getDebugInfo() const;

    // 内部目標車速の取得
    float getInternalTargetSpeed() const { return internalTargetSpeed; }


private:
    // PIDパラメータ保持用構造体
    struct PIDParams {
        float Kp;
        float Kp_High; // 目標車速から遠い場合（エラーが大きい場合）のPゲイン
        float Ki;
        float Ki_Decay; // 車速超過（減速方向）のIゲイン
        float Kd;
    };

    // ギアごとのPIDパラメータ (1速〜6速)
    // index 0: 1速, index 1: 2速 ... index 5: 6速
    PIDParams params[6];

    // PIDパラメータ定数（チューニング用）
    static constexpr float DEFAULT_KP       = 1.5f; // 通常Pゲイン (目標±INTEGRAL_ACTIVE_ZONE_KPH以内)
    static constexpr float DEFAULT_KP_HIGH  = 3.0f; // 遠距離用Pゲイン
    static constexpr float DEFAULT_KI       = 0.8f; // Iゲイン
    static constexpr float DEFAULT_KI_DECAY = 0.8f; // 車速超過(減速方向)のIゲイン
    static constexpr float DEFAULT_KD       = 0.5f; // Dゲイン

    // ベースライン（フィードフォワード値）
    static constexpr float BASELINE_PEDAL_PERCENT = 8.0f;

    // 不感帯 (デッドバンド) の幅 (km/h)
    // 目標速度の±この値 of 範囲内では、PI制御をホールドし現在の状態を維持する
    static constexpr float DEADBAND_KPH = 0.3f;

    // PWM出力上限・下限の定数
    static constexpr float CONTROL_MAX_PWM = 40.0f;               // 出力上限ペダル開度 (%)
    static constexpr float CONTROL_MIN_PWM = 1.0f;               // 通常時の最小ペダル開度 (%)
    static constexpr float OVERSHOOT_MIN_PWM_START_KPH = 1.0f;    // 引き下げ開始閾値 (km/h)
    static constexpr float OVERSHOOT_MIN_PWM_END_KPH = 2.0f;      // 0%化閾値 (km/h)


    // 積分項(I項)の蓄積を許可する速度差 (km/h) (アンチ・ワインドアップ用)
    // 目標速度から±この値より離れている時は、I項を蓄積しない（過剰なペダル保持を防ぐ）
    static constexpr float INTEGRAL_ACTIVE_ZONE_KPH = 1.0f;

    // 車速超過時のI項クランプ上限の動的引き下げ閾値 (km/h)
    // 実車速が目標車速をこの値以上超過すると、I項の上限を線形に引き下げ始める
    static constexpr float INTEGRAL_DECAY_START_KPH = 1.0f;
    // 実車速が目標車速をこの値以上超過すると、I項の上限を0.0%にする
    static constexpr float INTEGRAL_DECAY_END_KPH = 4.0f;
    // 車速超過時のI項動的制限の下限保証 (%)（オーバーシュート後の車速降下時の沈み込み防止）
    static constexpr float INTEGRAL_DECAY_FLOOR_PERCENT = 3.0f;

    // 車速降下時（減速トレンド時）の非対称不感帯の下限幅 (km/h)
    // 上から下がってきたときに目標車速割れ直前から早期にアクセル出力を立ち上げる
    static constexpr float DEADBAND_DESCENT_KPH = 0.05f;

    // 1制御周期(50ms)あたりで変化できる出力の最大量 (%)
    static constexpr float OUTPUT_MAX_CHANGE_PER_CYCLE = 0.5f; //5

    // 積分項の最大加算値制限
    static constexpr float INTEGRAL_MAX_PERCENT = 15.0f;

    // 目標車速ランプ制御の単位時間あたり変化レート (km/h per second)
    // 設定車速変更時の踏み込みショックを防ぐため、1秒間に最大3.0km/hの速度で内部目標速度を追従させます
    static constexpr float TARGET_RAMP_RATE_KPH_PER_SEC = 5.0f;

    // 内部状態
    float integral;
    float previousError;
    float lastOutput; // 前回の出力値（平滑化用）
    float internalTargetSpeed; // ランプ制御適用後の内部目標車速
    bool isTargetInitialized;
    float overrideHoldTimeRemaining; // オーバーライド検知後のランプアップ一時停止タイマー (秒)

    // 3秒間車速低下トレンドによるフィードフォワード(FF)ペダル加算パラメータ
    static constexpr float FF_MIN_ERROR_KPH   = 2.0f; // 発動目標落ち込み閾値 (km/h)
    static constexpr float FF_DROP_THRESH_KPH = 1.5f; // 3秒間での車速低下量閾値 (km/h)
    static constexpr float FF_GAIN            = 2.0f; // 低下量に対するペダル加算ゲイン
    static constexpr float FF_MAX_PERCENT     = 5.0f; // FFペダル加算量の上限 (%)
    static constexpr float FF_LOOKBACK_SEC    = 3.0f; // 低下量判定の過去参照時間 (秒)

    // 車速履歴リングバッファ (3秒前の車速取得用)
    static constexpr int HISTORY_SIZE = 100; // 50ms × 100 = 最大5秒分の履歴
    struct SpeedHistorySample {
        float time;
        float speed;
    };
    SpeedHistorySample speedHistory[HISTORY_SIZE];
    int historyHead;
    int historyCount;
    float elapsedTimeTotal;

    // OLED等のデバッグ表示用保持データ
    float lastPTerm;
    float lastITerm;
    float lastDTerm;
    float lastFFTerm;
    bool lastInDeadband;
    const char* lastLogicName;

    // 車速のローパスフィルタ用
    static constexpr float SPEED_FILTER_ALPHA = 0.05f; // フィルタ係数(0.0〜1.0) 小さいほど滑らか(遅い)
    float smoothedSpeed;
    float previousSmoothedSpeed; // 降下判定用（前回周期の平滑化車速）
    bool isSpeedInitialized;

    // 現在のギアに合わせたPIDパラメータを取得する
    PIDParams getParamsForGear(int gear) const;
};
