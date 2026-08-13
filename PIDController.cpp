#include "PIDController.h"
#include <cmath>

PIDController::PIDController() {
    // ギアごとのパラメータ初期化
    for (int i = 0; i < 6; i++) {
        params[i].Kp       = DEFAULT_KP;
        params[i].Kp_High  = DEFAULT_KP_HIGH;
        params[i].Ki       = DEFAULT_KI;
        params[i].Ki_Decay = DEFAULT_KI_DECAY;
        params[i].Kd       = DEFAULT_KD;
    }

    reset();
}

void PIDController::reset(float initialOutput) {
    integral = 0.0f;
    previousError = 0.0f;
    lastOutput = initialOutput;
    isSpeedInitialized = false;
    smoothedSpeed = 0.0f;
    previousSmoothedSpeed = 0.0f;
    historyHead = 0;
    historyCount = 0;
    elapsedTimeTotal = 0.0f;
    lastPTerm = 0.0f;
    lastITerm = 0.0f;
    lastDTerm = 0.0f;
    lastFFTerm = 0.0f;
    lastInDeadband = false;
    lastLogicName = "CTRL_OFF";
    isTargetInitialized = false;
    internalTargetSpeed = 0.0f;
    overrideHoldTimeRemaining = 0.0f;
}


PIDController::PIDParams PIDController::getParamsForGear(int gear) const {
    // ギア値 (10, 20, 30, 40, 50, 60) からインデックスを求める
    int index = (gear / 10) - 1;
    if (index < 0 || index > 5) {
        // 不正なギアや0の場合は、とりあえず1速のパラメータを使うなどフォールバック
        return params[0];
    }
    return params[index];
}

float PIDController::update(float currentSpeed, float targetSpeed, int gear, float dt, float accel) {
    // 1. 車速の平滑化 (EMA: Exponential Moving Average)
    if (!isSpeedInitialized) {
        smoothedSpeed = currentSpeed;
        previousSmoothedSpeed = currentSpeed;
        isSpeedInitialized = true;
    } else {
        previousSmoothedSpeed = smoothedSpeed;
        smoothedSpeed = smoothedSpeed * (1.0f - SPEED_FILTER_ALPHA) + currentSpeed * SPEED_FILTER_ALPHA;
    }

    // ギアが0（ニュートラルやクラッチ切り、停止中など）の場合はアクセルオフ（状態はHold）
    if (gear == 0) {
        lastLogicName = "CTRL_OFF";
        return 0.0f;
    }

    // dtが0以下の場合は計算をスキップ（ゼロ除算防止）
    if (dt <= 0.0f) {
        return BASELINE_PEDAL_PERCENT;
    }

    // 目標車速が0または極端に低い場合は、アクセルオフとみなす（安全対策）
    if (targetSpeed <= 0.1f) {
        reset();
        lastLogicName = "CTRL_OFF";
        return 0.0f;
    }


    // 目標車速のランプ制御（急激な設定車速変更時のショック防止）
    if (!isTargetInitialized) {
        internalTargetSpeed = smoothedSpeed; // 初期状態は現在の車速からスタート
        isTargetInitialized = true;
    }

    if (targetSpeed <= smoothedSpeed) {
        // 現在車速より低い（または等しい）場合はリミッター関係なし（即時追従）
        internalTargetSpeed = targetSpeed;
        overrideHoldTimeRemaining = 0.0f;
    } else {
        // 現在車速より高いところだけリミッターあり（ランプアップ制御）
        // 自車速が internalTargetSpeed を超えた場合（アクセル踏み込み等によるオーバーライド時）
        if (smoothedSpeed > internalTargetSpeed) {
            // リミッターに関係なく自車速の方が早い場合は、自車速＋レート分を設定（上限は targetSpeed）
            internalTargetSpeed = std::min(targetSpeed, smoothedSpeed + TARGET_RAMP_RATE_KPH_PER_SEC);
            
            // 1秒間のホールド期間を開始（または延長）
            overrideHoldTimeRemaining = 1.0f;
        } else {
            // 通常のランプアップ追従中 (smoothedSpeed <= internalTargetSpeed)
            if (overrideHoldTimeRemaining > 0.0f) {
                // ホールド期間中（1秒待機）
                overrideHoldTimeRemaining -= dt;
            } else {
                // ホールド期間終了後、目標車速に向けて TARGET_RAMP_RATE_KPH_PER_SEC でアップデート
                float max_target_change = TARGET_RAMP_RATE_KPH_PER_SEC * dt;
                if (targetSpeed > internalTargetSpeed + max_target_change) {
                    internalTargetSpeed += max_target_change;
                } else {
                    internalTargetSpeed = targetSpeed;
                }
            }
        }
    }


    // エラーの計算 (内部目標車速と平滑化された車速を使用)
    float raw_error = internalTargetSpeed - smoothedSpeed;

    // 車速降下トレンドの判定（車速が減少中、または目標を超過した状態から下がってきた場合）
    bool is_descending = (smoothedSpeed < previousSmoothedSpeed) || (raw_error < 0.0f);
    // 降下時はマイナス側（車速が目標を上回る側）の不感帯を浅く(DEADBAND_DESCENT_KPH)設定し、目標割れ直前から制御を開始
    float lower_deadband_kph = is_descending ? DEADBAND_DESCENT_KPH : DEADBAND_KPH;

    // ソフト・デッドバンド（不感帯）の適用
    float error = 0.0f;
    if (raw_error > DEADBAND_KPH) {
        error = raw_error - DEADBAND_KPH;
        lastInDeadband = false;
    } else if (raw_error < -lower_deadband_kph) {
        error = raw_error + lower_deadband_kph;
        lastInDeadband = false;
    } else {
        // 不感帯内では有効エラーを0とし、I項の更新をストップ（ホールド）する
        error = 0.0f;
        lastInDeadband = true;
    }

    // 現在のギアのPIDパラメータを取得
    PIDParams currentParams = getParamsForGear(gear);

    // 比例項 (P)
    // 折れ線ゲイン（アンチワインドアップのゾーン境界を境にゲインを切り替える）
    float pTerm = 0.0f;
    if (error > INTEGRAL_ACTIVE_ZONE_KPH) {
        pTerm = (currentParams.Kp * INTEGRAL_ACTIVE_ZONE_KPH) + (currentParams.Kp_High * (error - INTEGRAL_ACTIVE_ZONE_KPH));
    } else if (error < -INTEGRAL_ACTIVE_ZONE_KPH) {
        pTerm = (currentParams.Kp * -INTEGRAL_ACTIVE_ZONE_KPH) + (currentParams.Kp_High * (error + INTEGRAL_ACTIVE_ZONE_KPH));
    } else {
        pTerm = currentParams.Kp * error;
    }

    // 積分項 (I)
    // アンチ・ワインドアップ: 前回出力がリミット（30%上限または0%下限）に張り付いていて、
    // かつエラーがその張り付きをさらに強める方向にある場合は、これ以上のI項の蓄積を停止する。
    bool is_windup = false;
    if (lastOutput >= CONTROL_MAX_PWM && error > 0.0f) {
        is_windup = true;
    } else if (lastOutput <= 0.0f && error < 0.0f) {
        is_windup = true;
    }

    // エラー方向（加速・減速）に応じた積分ゲインの決定
    float current_ki = currentParams.Ki;
    if (error < 0.0f) {
        current_ki = currentParams.Ki_Decay;
    }

    if (!is_windup) {
        // 過去の蓄積に対してゲイン切り替えの影響が及ばないよう、
        // 蓄積時にゲインを掛け合わせる（I項連続化対策）
        integral += (error * current_ki) * dt;
    }

    float iTerm = integral;

    // 動的な積分項（I項）上限値の計算（車速超過量に応じた引き下げ）
    // アプローチB: INTEGRAL_DECAY_FLOOR_PERCENT(3.0%) を下限として保証し、過剰な削られによる落ち込みを防ぐ
    float dynamic_max_i = INTEGRAL_MAX_PERCENT;
    if (raw_error < -INTEGRAL_DECAY_START_KPH) {
        float overshoot = -raw_error;
        if (overshoot >= INTEGRAL_DECAY_END_KPH) {
            dynamic_max_i = INTEGRAL_DECAY_FLOOR_PERCENT;
        } else {
            // START値からEND値の間で線形に上限を引き下げる（下限をINTEGRAL_DECAY_FLOOR_PERCENTとする）
            float ratio = (overshoot - INTEGRAL_DECAY_START_KPH) / (INTEGRAL_DECAY_END_KPH - INTEGRAL_DECAY_START_KPH);
            dynamic_max_i = INTEGRAL_MAX_PERCENT - ratio * (INTEGRAL_MAX_PERCENT - INTEGRAL_DECAY_FLOOR_PERCENT);
        }
    }

    // 積分項の制限 (ワインドアップ対策 & 動的上限)
    if (iTerm > dynamic_max_i) {
        iTerm = dynamic_max_i;
        integral = dynamic_max_i;
    } else if (iTerm < -INTEGRAL_MAX_PERCENT) {
        iTerm = -INTEGRAL_MAX_PERCENT;
        integral = -INTEGRAL_MAX_PERCENT;
    }

    // 車速履歴リングバッファの更新 (過去3秒間の速度推移判定用)
    elapsedTimeTotal += dt;
    speedHistory[historyHead] = { elapsedTimeTotal, smoothedSpeed };
    historyHead = (historyHead + 1) % HISTORY_SIZE;
    if (historyCount < HISTORY_SIZE) {
        historyCount++;
    }

    // 微分項 (D)
    float derivative = (error - previousError) / dt;
    float dTerm = currentParams.Kd * derivative;

    // 過去約3.0秒前の車速を取得
    float targetTime = elapsedTimeTotal - FF_LOOKBACK_SEC;
    float speed_3s_ago = smoothedSpeed;
    if (historyCount > 1 && targetTime > 0.0f) {
        float min_dt = 9999.0f;
        for (int i = 0; i < historyCount; i++) {
            float sample_dt = std::abs(speedHistory[i].time - targetTime);
            if (sample_dt < min_dt) {
                min_dt = sample_dt;
                speed_3s_ago = speedHistory[i].speed;
            }
        }
    }

    // 3秒間での車速低下量
    float speed_drop_3s = speed_3s_ago - smoothedSpeed;

    // フィードフォワード(FF)加速度/トレンド補正項の計算
    // 条件1: 車速が目標車速より 2.0 km/h 以上落ち込んでいる (raw_error >= 2.0kph)
    // 条件2: 過去3秒間での車速低下量が 1.5 km/h 以上である (減速トレンドが継続)
    float ffTerm = 0.0f;
    if (raw_error >= FF_MIN_ERROR_KPH && speed_drop_3s >= FF_DROP_THRESH_KPH) {
        ffTerm = std::min(FF_MAX_PERCENT, std::max(0.0f, (speed_drop_3s - 1.0f) * FF_GAIN));
    }

    // ベースライン（FF項）の計算
    float current_base = BASELINE_PEDAL_PERCENT;

    // 出力の計算 (ベースライン + P + I + D + 車速低下FF)
    float output = current_base + pTerm + iTerm + dTerm + ffTerm;

    // 1周期で変化できる量を制限（急変に対するローパスフィルタ）
    if (output - lastOutput > OUTPUT_MAX_CHANGE_PER_CYCLE) {
        output = lastOutput + OUTPUT_MAX_CHANGE_PER_CYCLE;
    } else if (output - lastOutput < -OUTPUT_MAX_CHANGE_PER_CYCLE) {
        output = lastOutput - OUTPUT_MAX_CHANGE_PER_CYCLE;
    }

    // エンジンブレーキを使わないため、最小PWM出力は常に CONTROL_MIN_PWM (3.0%) を下限とする
    float dynamic_min_pwm = CONTROL_MIN_PWM;

    // 出力制限 (dynamic_min_pwm% 〜 CONTROL_MAX_PWM%)
    if (output > CONTROL_MAX_PWM) {
        output = CONTROL_MAX_PWM;
    } else if (output < dynamic_min_pwm) {
        output = dynamic_min_pwm;
    }

    lastOutput = output; // 次回用の出力保存

    // デバッグ表示用の保存
    lastPTerm = pTerm;
    lastITerm = iTerm;
    lastDTerm = dTerm;
    lastFFTerm = ffTerm;

    // 現在のアクティブ制御ロジック名の決定
    if (lastInDeadband) {
        lastLogicName = "DEADBAND";
    } else if (ffTerm > 0.0f) {
        lastLogicName = "FF_TREND";
    } else if (std::abs(error) > INTEGRAL_ACTIVE_ZONE_KPH) {
        lastLogicName = "P_HIGH";
    } else if (raw_error < -INTEGRAL_DECAY_START_KPH) {
        lastLogicName = "I_DECAY";
    } else if (targetSpeed > smoothedSpeed && (overrideHoldTimeRemaining > 0.0f || internalTargetSpeed < targetSpeed - 0.1f)) {
        lastLogicName = "RAMP_UP";
    } else {
        lastLogicName = "PID_NORM";
    }

    // 次回用のエラー保存
    previousError = error;

    return output;
}

PIDController::PIDDebugInfo PIDController::getDebugInfo() const {
    PIDDebugInfo info;
    info.pTerm = lastPTerm;
    info.iTerm = lastITerm;
    info.dTerm = lastDTerm;
    info.ffTerm = lastFFTerm;
    info.inDeadband = lastInDeadband;
    info.logicName = lastLogicName;
    return info;
}


void PIDController::syncIntegral(float speed, float target_speed, int gear, float targetOutput) {
    if (gear == 0) {
        return;
    }

    // 1) 入力値のクリップガード
    if (targetOutput > CONTROL_MAX_PWM) {
        targetOutput = CONTROL_MAX_PWM;
    } else if (targetOutput < 0.0f) {
        targetOutput = 0.0f;
    }

    // 2) 比例項（P項）の計算
    PIDParams currentParams = getParamsForGear(gear);
    if (!isSpeedInitialized) {
        smoothedSpeed = speed;
        previousSmoothedSpeed = speed;
        isSpeedInitialized = true;
    }
    float raw_error = target_speed - smoothedSpeed;
    float error = 0.0f;
    if (raw_error > DEADBAND_KPH) {
        error = raw_error - DEADBAND_KPH;
    } else if (raw_error < -DEADBAND_KPH) {
        error = raw_error + DEADBAND_KPH;
    } else {
        error = 0.0f;
    }
    float pTerm = currentParams.Kp * error;

    // 3) ベースライン（FF項）の計算
    // update関数と同様に、エラーによる急激な0カットを行わず固定値をベースとする。
    float current_base = BASELINE_PEDAL_PERCENT;

    // 4) 必要となる積分項（I項）の逆算
    float requiredITerm = targetOutput - (current_base + pTerm);

    // 5) 積分値（integral 変数）の上書き
    integral = requiredITerm;

    // 同期した際、lastOutputも同期先の出力に合わせておくことで、次回のローパス制限で引っかかるのを防ぐ
    lastOutput = targetOutput;

    // 動的な上限値の計算 (アプローチB: INTEGRAL_DECAY_FLOOR_PERCENT を適用)
    float dynamic_max_i = INTEGRAL_MAX_PERCENT;
    if (raw_error < -INTEGRAL_DECAY_START_KPH) {
        float overshoot = -raw_error;
        if (overshoot >= INTEGRAL_DECAY_END_KPH) {
            dynamic_max_i = INTEGRAL_DECAY_FLOOR_PERCENT;
        } else {
            float ratio = (overshoot - INTEGRAL_DECAY_START_KPH) / (INTEGRAL_DECAY_END_KPH - INTEGRAL_DECAY_START_KPH);
            dynamic_max_i = INTEGRAL_MAX_PERCENT - ratio * (INTEGRAL_MAX_PERCENT - INTEGRAL_DECAY_FLOOR_PERCENT);
        }
    }

    // 6) 積分値のアンチワインドアップ制限（動的上限を適用）
    if (integral > dynamic_max_i) {
        integral = dynamic_max_i;
    } else if (integral < -INTEGRAL_MAX_PERCENT) {
        integral = -INTEGRAL_MAX_PERCENT;
    }

    // 7) ローパスフィルタの連続性維持
    lastOutput = targetOutput;
}
