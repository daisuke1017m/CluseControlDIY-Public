#pragma once

#include <Arduino.h>

/**
 * DataInput クラス
 * CAN(TWAI)通信を利用して、ELM327経由で車両データ（車速、RPM、スロットル、ペダルD）を取得し、
 * スケール変換やギアの推定を行う機能を提供します。
 */
class DataInput {
public:
    // コンストラクタ
    DataInput();

    // 初期化メソッド（CANのセットアップを行います）
    void init();

    // 毎ループ呼び出し、CAN通信の送受信を非同期で進めます
    void update();

    // --- 取得したデータを読み出すメソッド ---
    float getSpeed()          const;                                       // 車速 [km/h] ：CAN車速を使用
    float getPulseSpeed()     const { return filteredPulseSpeed; }         // 車輪速パルス車速 [km/h]（ノイズ解析用・フィルタ済）
    float getCanSpeed()       const { return currentSpeed; }               // CAN車速 [km/h]
    unsigned long getPulsePeriodUs() const { return pulsePeriodUs; }       // 最新パルス間隔 [us]（ノイズ解析用）
    float getRPM() const { return currentRPM; }
    float getThrottle() const { return currentThrottle; }
    float getPedal() const { return currentPedal; }
    float getLoad() const { return currentLoad; }
    int getGear() const { return currentGear; }
    bool getBrake() const { return currentBrake; }
    bool getClutch() const { return currentClutch; }
    
    // CANエラー状態の取得
    bool isCanError() const { return canErrorState; }

    // CAN通信が実際に有効（エラーがなく、データも受信できている）か
    bool isCanActive() const { return !canErrorState && hasReceivedData; }

    // ISRから更新するため static volatile（.cpp内のフリー関数ISRからアクセス）
    static volatile unsigned long latestPulseTime_s;
    static volatile unsigned long latestPulseInterval_s; // ISRが算出した隣接パルス間隔 [us]
    static volatile bool          newPulseOccurred_s;

private:
    // 車両状態データ
    float currentSpeed;
    float currentRPM;
    float currentThrottle;
    float currentPedal;
    float currentLoad;
    int currentGear; // 10, 20, 30, 40, 50, 60 (0は不明または停止中)
    bool currentBrake;  // ブレーキ信号 (CAN 0x512から取得)
    bool currentClutch; // クラッチ信号 (CAN 0x600 data[5] bit2から取得)
    bool canErrorState; // CANエラー状態
    bool hasReceivedData; // 一度でもCANを受信したか

    // ポーリング制御用ステート
    const uint8_t plotPids[5] = {0x0D, 0x0C, 0x11, 0x49, 0x04}; // 車速, RPM, スロットル, ペダルD, EngineLoad
    int currentPlotPidIndex;
    bool waitingForResponse;
    unsigned long lastRequestTime;
    unsigned long lastPlotCycleStartTime;

    // CAN車速10回移動平均用
    static const int SPEED_MA_WINDOW = 10;
    float speedBuffer[SPEED_MA_WINDOW];
    int speedBufferIndex;
    int speedSampleCount;
    float speedSum;
    
    // 定数
    static const unsigned long PLOT_CYCLE_INTERVAL = 50; // 全体の取得周期(ミリ秒)
    static const unsigned long TIMEOUT_MS = 100;         // レスポンス待ちのタイムアウト

    // --- 車輪速パルス計測 ---
    float         pulseSpeed;         // パルス計測生車速 [km/h]
    float         filteredPulseSpeed; // ±5km/hノイズフィルタ適用後のパルス車速 [km/h]
    uint8_t       consecutiveRejectCount; // 連続拒絶回数カウンタ（安全復帰用）
    unsigned long pulsePeriodUs;      // 直前の有効パルス間隔 [us]（ノイズ解析用）
    unsigned long lastValidPulseTime; // 最後に有効と判定されたパルスの発生時刻 [us]
    unsigned long lastValidInterval;  // 直前の有効間隔（ヒゲ棄却用） [us]

    // パルス計測処理（update()内から呼び出す）
    void updatePulse();

    // 正規化パラメータ
    static constexpr float PEDAL_MIN = 13.33f;
    static constexpr float PEDAL_MAX = 65.49f;
    static constexpr float THROTTLE_MIN = 17.25f;
    static constexpr float THROTTLE_MAX = 78.82f;

    // ギア推定用パラメータ
    static constexpr float GEAR1_RATIO = 118.0f;
    static constexpr float GEAR1_DELTA = 10.0f;
    static constexpr float GEAR2_RATIO = 75.0f;
    static constexpr float GEAR2_DELTA = 5.0f;
    static constexpr float GEAR3_RATIO = 54.0f;
    static constexpr float GEAR3_DELTA = 5.0f;
    static constexpr float GEAR4_RATIO = 43.0f;
    static constexpr float GEAR4_DELTA = 5.0f;
    static constexpr float GEAR5_RATIO = 34.0f;
    static constexpr float GEAR5_DELTA = 3.0f;
    static constexpr float GEAR6_RATIO = 27.0f;
    static constexpr float GEAR6_DELTA = 3.0f;

    // ギア推定処理
    void estimateGear();
    
    // スケール変換処理
    float normalizePedal(float rawValue);
    float normalizeThrottle(float rawValue);
};
