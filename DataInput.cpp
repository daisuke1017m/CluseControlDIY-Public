#include "DataInput.h"
#include "Config.h"
#include "driver/twai.h"

// =============================================
// ISR共有用 staticメンバ変数の定義
// =============================================
volatile unsigned long DataInput::latestPulseTime_s  = 0;
volatile unsigned long DataInput::latestPulseInterval_s = 0;
volatile bool          DataInput::newPulseOccurred_s = false;

// =============================================
// 車輪速パルス ISR（IRAM配置・フリー関数）
// =============================================
void IRAM_ATTR speedPulseISR() {
    unsigned long now = micros();
    if (DataInput::latestPulseTime_s != 0) {
        DataInput::latestPulseInterval_s = now - DataInput::latestPulseTime_s;
    }
    DataInput::latestPulseTime_s  = now;
    DataInput::newPulseOccurred_s = true;
}

DataInput::DataInput()
    : currentSpeed(0), currentRPM(0), currentThrottle(0), currentPedal(0), currentLoad(0), currentGear(0),
      currentBrake(false), currentClutch(false), canErrorState(false), hasReceivedData(false),
      currentPlotPidIndex(0), waitingForResponse(false),
      lastRequestTime(0), lastPlotCycleStartTime(0),
      speedBufferIndex(0), speedSampleCount(0), speedSum(0.0f),
      pulseSpeed(0.0f), filteredPulseSpeed(0.0f), consecutiveRejectCount(0),
      pulsePeriodUs(0), lastValidPulseTime(0), lastValidInterval(0) {
    for (int i = 0; i < SPEED_MA_WINDOW; i++) {
        speedBuffer[i] = 0.0f;
    }
}

void DataInput::init() {
    // 1. CAN全般設定：双方向通信（NORMAL）モード
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        ESP32_CAN_TX_PIN, 
        ESP32_CAN_RX_PIN, 
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 500;

    // 2. 通信速度の設定：500kbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // 3. フィルターの設定：すべてのメッセージを通す (ACCEPT_ALL)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 初期化と開始
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("DataInput: CANドライバのインストールに失敗しました");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("DataInput: CAN通信の開始に失敗しました");
        return;
    }
    Serial.println("DataInput: CAN initialized.");

    // 4. 車輪速パルスピン設定（トランジスタ反転回路経由、RISINGエッジで検出）
    pinMode(VEHICLE_SPEED_PULSE_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(VEHICLE_SPEED_PULSE_PIN), speedPulseISR, RISING);
    Serial.println("DataInput: Pulse speed initialized on GPIO32 (RISING).");
}

void DataInput::update() {
    // 0. 車輪速パルス計測の更新（毎ループ実行）
    updatePulse();

    // 0.5. TWAI（CAN）のステータス監視とリカバリ処理
    twai_status_info_t status_info;
    if (twai_get_status_info(&status_info) == ESP_OK) {
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            // バスオフ状態ならリカバリを開始
            twai_initiate_recovery();
            canErrorState = true;
            Serial.println("CAN Error: Bus-Off detected. Initiating recovery...");
        } else if (status_info.state == TWAI_STATE_RECOVERING) {
            // リカバリ中
            canErrorState = true;
        } else if (status_info.state == TWAI_STATE_STOPPED) {
            // リカバリ完了後、または停止中の場合再スタート
            twai_start();
            canErrorState = true; // 次のループでRUNNINGになるまでエラー扱い
            Serial.println("CAN Recovery complete. Restarting...");
        } else if (status_info.state == TWAI_STATE_RUNNING) {
            // 正常稼働
            canErrorState = false;
        }
    }

    // 1. 受信バッファにあるメッセージをすべて読み出す（ノンブロッキング）
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        hasReceivedData = true; // 初回データ受信フラグ

        // ブレーキ信号 (0x512) の処理
        if (rx_msg.identifier == 0x512) {
            if (rx_msg.data_length_code >= 5) {
                currentBrake = (rx_msg.data[4] & 0x10) != 0;
            }
        }else if(rx_msg.identifier == 0x600){
            if(rx_msg.data_length_code >=6){
                currentClutch = (rx_msg.data[6] & 0x04) == 0;
            }
        }
        // Mode 0x41 応答 かつ 要求したPIDか確認 (OBD2 レスポンス)
        else if (waitingForResponse && rx_msg.identifier >= 0x7E0 && rx_msg.identifier <= 0x7FF && 
                 rx_msg.data_length_code >= 3 && rx_msg.data[1] == 0x41 && rx_msg.data[2] == plotPids[currentPlotPidIndex]) {
            float val = 0;
            uint8_t pid = rx_msg.data[2];
            if (pid == 0x0D) { // 車速
                val = rx_msg.data[3];
                // CAN車速の10回移動平均処理
                speedSum -= speedBuffer[speedBufferIndex];
                speedBuffer[speedBufferIndex] = val;
                speedSum += val;
                speedBufferIndex = (speedBufferIndex + 1) % SPEED_MA_WINDOW;
                if (speedSampleCount < SPEED_MA_WINDOW) {
                    speedSampleCount++;
                }
                currentSpeed = speedSum / (float)speedSampleCount;
            } else if (pid == 0x0C) { // RPM
                val = (rx_msg.data[3] * 256.0 + rx_msg.data[4]) / 4.0;
                currentRPM = val;
                // 車速とRPMが揃うタイミングなどでギア推定を更新
                estimateGear();
            } else if (pid == 0x11) { // スロットル
                val = rx_msg.data[3] * 100.0 / 255.0;
                currentThrottle = normalizeThrottle(val);
            } else if (pid == 0x49) { // ペダルD
                val = rx_msg.data[3] * 100.0 / 255.0;
                currentPedal = normalizePedal(val);
            } else if (pid == 0x04) { // Calculated engine load
                val = rx_msg.data[3] * 100.0 / 255.0;
                currentLoad = val;
            }
            
            // 次のPIDへ
            currentPlotPidIndex++;
            waitingForResponse = false;
            
            if (currentPlotPidIndex >= 5) {
                currentPlotPidIndex = 0;
            }
        }
    }

    // 2. タイムアウト処理
    if (waitingForResponse && (millis() - lastRequestTime > TIMEOUT_MS)) {
        waitingForResponse = false;
        currentPlotPidIndex++;
        if (currentPlotPidIndex >= 5) {
            currentPlotPidIndex = 0;
        }
    }

    // 3. 次のリクエストを送信する
    if (!waitingForResponse) {
        // 全体を周期的に取得するための制御
        if (currentPlotPidIndex == 0 && (millis() - lastPlotCycleStartTime < PLOT_CYCLE_INTERVAL)) {
            // まだ周期が経過していないのでスキップ
            return;
        }

        if (currentPlotPidIndex == 0) {
            lastPlotCycleStartTime = millis();
        }

        // 次のPIDをリクエスト送信
        twai_message_t tx_msg;
        memset(&tx_msg, 0, sizeof(tx_msg));
        tx_msg.identifier = 0x7DF;
        tx_msg.extd = 0;
        tx_msg.rtr = 0;
        tx_msg.data_length_code = 8;
        tx_msg.data[0] = 0x02;
        tx_msg.data[1] = 0x01; // Mode 01
        tx_msg.data[2] = plotPids[currentPlotPidIndex];
        
        if (twai_transmit(&tx_msg, pdMS_TO_TICKS(5)) == ESP_OK) {
            waitingForResponse = true;
            lastRequestTime = millis();
        }
    }
}

float DataInput::normalizePedal(float rawValue) {
    // ペダル開度正規化
    float normalized = (rawValue - PEDAL_MIN) / (PEDAL_MAX - PEDAL_MIN) * 100.0f;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 100.0f) normalized = 100.0f;
    return normalized;
}

float DataInput::normalizeThrottle(float rawValue) {
    // スロットル開度正規化
    float normalized = (rawValue - THROTTLE_MIN) / (THROTTLE_MAX - THROTTLE_MIN) * 100.0f;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 100.0f) normalized = 100.0f;
    return normalized;
}


// =============================================
// 車速取得（パルス車速一本化）
// =============================================
float DataInput::getSpeed() const {
    return currentSpeed; // CAN車速を制御に使用
}

// =============================================
// 車輪速パルス計測処理
// =============================================
void DataInput::updatePulse() {
    if (newPulseOccurred_s) {
        // ISR共有変数をアトミックに読み取り、フラグをクリア
        noInterrupts();
        unsigned long currentPulseTime = latestPulseTime_s;
        unsigned long currentInterval  = latestPulseInterval_s;
        newPulseOccurred_s             = false;
        interrupts();

        // 停止状態（基準点が未設定）からの最初のパルス：基準時刻を記録して抜ける
        if (lastValidPulseTime == 0) {
            lastValidPulseTime = currentPulseTime;
        } else if (currentInterval > 0) {
            unsigned long diff = currentInterval;

            // 200km/h 上限カット：7000us以下（≒201.8km/h超）は無条件棄却
            if (diff > 7000) {
                // 突発ヒゲ棄却フィルタ（10km/h以上の走行中のみ有効）
                // 前回有効間隔の85%未満に急激短縮するパルス → ノイズと判定
                bool isValidPulse = true;
                if (lastValidInterval > 0 && lastValidInterval < 141287UL) {
                    if (diff < (lastValidInterval * 85 / 100)) {
                        isValidPulse = false;
                    }
                    // 欠落パルス・ノイズ対策: 前回間隔の3倍以上に一気に伸びる場合は棄却
                    else if (diff > (lastValidInterval * 3)) {
                        isValidPulse = false;
                    }
                }

                if (isValidPulse) {
                    // JIS D 1601 4パルス仕様: V [km/h] = 1412873 / T [us]
                    lastValidInterval  = diff;
                    pulsePeriodUs      = diff;
                    pulseSpeed         = 1412873.0f / (float)diff;
                    lastValidPulseTime = currentPulseTime;

                    // ±5km/h 差分フィルタ（リジェクションフィルタ）
                    if (filteredPulseSpeed == 0.0f) {
                        // 停車時・発進時・タイムアウト復帰時は即時更新
                        filteredPulseSpeed = pulseSpeed;
                        consecutiveRejectCount = 0;
                    } else {
                        float diffSpeed = fabs(pulseSpeed - filteredPulseSpeed);
                        if (diffSpeed <= 5.0f || consecutiveRejectCount >= 5) {
                            // ±5km/h以内の正常値、または連続ノイズ判定5回以上の安全復帰
                            filteredPulseSpeed = pulseSpeed;
                            consecutiveRejectCount = 0;
                        } else {
                            // 5km/h超のヒゲノイズは棄却（前回値を保持）
                            consecutiveRejectCount++;
                        }
                    }
                }
            }
        }
    }

    // 2秒タイムアウト：パルスが来なければ停車とみなしリセット
    // リセット後にパルスが来たら自動復帰
    {
        noInterrupts();
        unsigned long lastT = latestPulseTime_s;
        interrupts();

        if (lastT == 0 || (micros() - lastT > 2000000UL)) {
            pulseSpeed         = 0.0f;
            filteredPulseSpeed = 0.0f;
            consecutiveRejectCount = 0;
            pulsePeriodUs      = 0;
            lastValidInterval  = 0;
            lastValidPulseTime = 0;
            noInterrupts();
            latestPulseTime_s = 0;
            latestPulseInterval_s = 0;
            interrupts();
        }
    }
}

void DataInput::estimateGear() {
    // ギア推定はCAN車速を使用する（ノイズの影響を受けにくい）
    if (currentSpeed <= 0.1f) {
        currentGear = 0; // 停止時は0
        return;
    }
    
    float ratio = currentRPM / currentSpeed;
    
    // ギア推定判定 (abs を使用)
    if (abs(ratio - GEAR1_RATIO) <= GEAR1_DELTA) {
        currentGear = 10;
    } else if (abs(ratio - GEAR2_RATIO) <= GEAR2_DELTA) {
        currentGear = 20;
    } else if (abs(ratio - GEAR3_RATIO) <= GEAR3_DELTA) {
        currentGear = 30;
    } else if (abs(ratio - GEAR4_RATIO) <= GEAR4_DELTA) {
        currentGear = 40;
    } else if (abs(ratio - GEAR5_RATIO) <= GEAR5_DELTA) {
        currentGear = 50;
    } else if (abs(ratio - GEAR6_RATIO) <= GEAR6_DELTA) {
        currentGear = 60;
    } else {
        // 該当なし（クラッチを切っている、半クラなど）
        currentGear = 0; 
    }
}
