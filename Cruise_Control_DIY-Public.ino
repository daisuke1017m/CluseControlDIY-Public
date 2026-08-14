#include <Arduino.h>
#include "Config.h"
#include "DataInput.h"
#include "PIDController.h"
#include "PedalIO.h"
#include "OledDisplay.h"
#include "AutoLight.h"

// インスタンス作成
DataInput dataInput;
PIDController pidController;
PedalIO pedalIO;
OledDisplay oled;
AutoLight autoLight;

// 制御状態変数
bool controlEnabled = false; // タクトスイッチでONになるとPID制御を行う

// 安全機構（キルスイッチ）状態変数
bool killSwitchActivated = false;

// OLED表示切替スイッチ（GPIO33）チャタリング防止用変数
bool lastConfirmedOledSwState = HIGH;
int oledSwPressCounter = 0;

// クルーズ操作キャンセル状態フラグ
bool lastConfirmedCruiseCancelState = false;

// UP/DOWNアナログスイッチ状態変数
enum UpDownState { UDS_NONE, UDS_UP, UDS_DOWN };
UpDownState currentUpDownState = UDS_NONE;
UpDownState lastUpDownState = UDS_NONE;
unsigned long upDownPressStartTime = 0;
unsigned long upDownLastContinuousTime = 0;
bool upDownLongPressInitialDone = false;

// 状態変数
float targetSpeed = 0.0f; // 0.0fは未設定状態を意味する
unsigned long lastControlTime = 0;

// ギアシフト検出用状態変数
int  gearAtClutchPress  = 0;    // クラッチを踏んだ瞬間に記録したギア
bool prevClutchReleased = true; // 前周期のクラッチ状態（Press エッジ検出用）

// シリアル入力用バッファ
String inputString = "";

// ─────────────────────────────────────────────────────────────
// OLED描画タスク (Core 0 で動作)
// ─────────────────────────────────────────────────────────────
void oledTask(void *pvParameters) {
    for (;;) {
        oled.refresh();
        vTaskDelay(pdMS_TO_TICKS(50)); // 20fps
    }
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    while (!Serial) { ; }
    
    Serial.println("Cruise Control Simulation Ready.");
    Serial.println("Please input Target Speed (km/h) via Serial. (e.g. 60)");
    
    // キルスイッチ用ピン初期化（LOWで開始）
    pinMode(KILL_SWITCH_PIN, OUTPUT);
    digitalWrite(KILL_SWITCH_PIN, LOW);

    // HMIピン初期化
    pinMode(OLED_MODE_SWITCH_PIN, INPUT_PULLUP); // GPIO33: OLED表示切替スイッチ
    pinMode(CRUISE_SW_PIN, INPUT);              // GPIO14: クルーズ操作スイッチ（ADC抵抗分圧）

    // 各種初期化
    dataInput.init();
    pedalIO.init();
    oled.init();
    autoLight.init();
    
    // OLED専用タスクをCore 0で起動
    xTaskCreatePinnedToCore(oledTask, "OLED_Task", 4096, NULL, 1, NULL, 0);
    
    lastControlTime = millis();
}

void loop() {
    // --- 安全機構：起動後一定時間経過でキルスイッチ用トランジスタをON ---
    if (!killSwitchActivated && millis() >= KILL_SWITCH_DELAY_MS) {
        digitalWrite(KILL_SWITCH_PIN, HIGH);
        killSwitchActivated = true;
        Serial.println("Kill Switch Relay ON");
    }

    // --- PCからのシリアル入力を監視（目標車速の設定） ---
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        
        if (inChar == '\r' || inChar == '\n') {
            inputString.trim();
            if (inputString.length() > 0) {
                // 入力文字列をfloatに変換して目標車速に設定
                targetSpeed = inputString.toFloat();
                if (targetSpeed > 0.0f) {
                    targetSpeed = max(MIN_TARGET_SPEED, targetSpeed);
                }
                Serial.print("Target Speed set to: ");
                Serial.println(targetSpeed);
                // 目標が変わったのでPIDの積分などをリセット
                pidController.reset(pedalIO.readMainPedal());
            }
            inputString = "";
        } else {
            inputString += inChar;
        }
    }

    // --- データ入力（CAN通信）の更新 ---
    dataInput.update();

    // --- 20msec 周期のPID制御とシリアル出力 ---
    unsigned long currentTime = millis();
    if (currentTime - lastControlTime >= CONTROL_CYCLE_MS) {
        float dt = (currentTime - lastControlTime) / 1000.0f; // 秒単位
        lastControlTime = currentTime;


        // CAN通信エラー状態とブレーキスイッチ状態の読み取り
        bool brakeActive = dataInput.getBrake(); // CANからのブレーキ信号を使用
        bool canErrorActive = dataInput.isCanError(); // CAN通信エラー状態

        // AutoLight 更新 (CAN車速, CAN通信有効状態, 経過時間から距離を計算して点灯制御)
        bool canActive = dataInput.isCanActive(); // CANデータ受信済み＆エラーなし
        autoLight.update(dataInput.getCanSpeed(), canActive, dt);

        if (brakeActive || canErrorActive) {
            controlEnabled = false;
        }

        // 2. HMIスイッチの読み取りと処理

        // --- クルーズコントロール操作スイッチ (GPIO14) の読み取り ---
        // 3.3V : SW何もなし (NONE)  [>= 2.8V]
        // 0V   : Cancel            [< 0.5V]
        // 1.07V: Res/UP            [0.5V 〜 1.6V]
        // 2.25V: Set/Down          [1.6V 〜 2.8V]
        int cruiseSwRawAdc = analogRead(CRUISE_SW_PIN);
        float cruiseSwVoltage = (cruiseSwRawAdc / 4095.0f) * 3.3f;
        UpDownState newUpDownState = UDS_NONE;
        bool cruiseCancelPressed = false;

        if (cruiseSwVoltage < 0.5f) {
            // 0V 付近 -> Cancel
            cruiseCancelPressed = true;
        } else if (cruiseSwVoltage >= 0.5f && cruiseSwVoltage < 1.6f) {
            // 1.07V 付近 -> Res/UP
            newUpDownState = UDS_UP;
        } else if (cruiseSwVoltage >= 1.6f && cruiseSwVoltage < 2.8f) {
            // 2.25V 付近 -> Set/Down
            newUpDownState = UDS_DOWN;
        }

        // Cancelスイッチの押下判定（エッジ検出）
        if (!lastConfirmedCruiseCancelState && cruiseCancelPressed) {
            // Cancel押下エッジ: 制御OFF（Target車速は維持）
            controlEnabled = false;
        }
        lastConfirmedCruiseCancelState = cruiseCancelPressed;

        // Res/UP・Set/Down スイッチ判定
        if (newUpDownState != currentUpDownState) {
            currentUpDownState = newUpDownState;
            if (currentUpDownState != UDS_NONE) {
                // 押された瞬間（エッジ）
                upDownPressStartTime = currentTime;
                upDownLongPressInitialDone = false;

                if (currentUpDownState == UDS_UP) {
                    if (!controlEnabled && targetSpeed > 0.0f) {
                        // Cancel中 → 設定車速にResume（制御ON）
                        if (!brakeActive && !canErrorActive) {
                            controlEnabled = true;
                            pidController.reset(pedalIO.readMainPedal());
                        }
                    } else if (controlEnabled && targetSpeed > 0.0f) {
                        // 制御中・短押し (1回クリック) → +1kph
                        targetSpeed += 1.0f;
                        targetSpeed = max(MIN_TARGET_SPEED, targetSpeed);
                    }
                } else if (currentUpDownState == UDS_DOWN) {
                    if (!controlEnabled) {
                        // Cancel中 → 現在の車速でSet＋制御ON
                        if (!brakeActive && !canErrorActive) {
                            float spd = dataInput.getSpeed();
                            targetSpeed = max(MIN_TARGET_SPEED, spd);
                            controlEnabled = true;
                            pidController.reset(pedalIO.readMainPedal());
                        }
                    } else if (controlEnabled && targetSpeed > 0.0f) {
                        // 制御中・短押し (1回クリック) → -1kph
                        targetSpeed -= 1.0f;
                        targetSpeed = max(MIN_TARGET_SPEED, targetSpeed);
                    }
                }
            }
        } else if (currentUpDownState != UDS_NONE) {
            // 押しっぱなし処理（制御中のみ有効）
            if (!upDownLongPressInitialDone && (currentTime - upDownPressStartTime >= 1000)) {
                // 1秒長押しの初回アクション: 5の倍数へ丸める
                if (controlEnabled && targetSpeed > 0.0f) {
                    if (currentUpDownState == UDS_UP) {
                        float rounded = ceil(targetSpeed / 5.0f) * 5.0f;
                        targetSpeed = (rounded > targetSpeed) ? rounded : targetSpeed + 5.0f;
                    } else if (currentUpDownState == UDS_DOWN) {
                        float rounded = floor(targetSpeed / 5.0f) * 5.0f;
                        targetSpeed = (rounded < targetSpeed) ? rounded : targetSpeed - 5.0f;
                    }
                    targetSpeed = max(MIN_TARGET_SPEED, targetSpeed);
                    oled.triggerTargetSpeedInvert(); // 5kph変更を反転表示通知
                }
                upDownLongPressInitialDone = true;
                upDownLastContinuousTime = currentTime;
            } else if (upDownLongPressInitialDone && (currentTime - upDownLastContinuousTime >= 500)) {
                // 以降、500msごとに5kph変更
                if (controlEnabled && targetSpeed > 0.0f) {
                    if (currentUpDownState == UDS_UP) {
                        targetSpeed += 5.0f;
                    } else if (currentUpDownState == UDS_DOWN) {
                        targetSpeed -= 5.0f;
                    }
                    targetSpeed = max(MIN_TARGET_SPEED, targetSpeed);
                    oled.triggerTargetSpeedInvert(); // 5kph変更を反転表示通知
                }
                upDownLastContinuousTime = currentTime;
            }
        }

        // --- OLED表示モード切替スイッチ (GPIO33) ---
        int modeSwRaw = digitalRead(OLED_MODE_SWITCH_PIN);
        bool currentOledSwState = lastConfirmedOledSwState;
        if (modeSwRaw == LOW) {
            oledSwPressCounter++;
            if (oledSwPressCounter >= 3) { currentOledSwState = LOW; }
        } else {
            oledSwPressCounter = 0;
            currentOledSwState = HIGH;
        }

        if (lastConfirmedOledSwState == HIGH && currentOledSwState == LOW) {
            // モード切替スイッチ押下エッジ: OLEDの表示モードをサイクリック切り替え
            oled.nextDisplayMode();
        }
        lastConfirmedOledSwState = currentOledSwState;

        // OEM仕様：Cancel中はtargetSpeedを維持するため、50%以下リセットは行わない

        // 最新の車両データを取得
        float currentSpeed  = dataInput.getSpeed();      // 車速 [km/h] CAN車速を使用
        float pulseSpeedVal = dataInput.getPulseSpeed(); // パルス車速 [km/h]（ノイズ解析用）
        float canSpeedVal   = dataInput.getCanSpeed();   // CAN車速 [km/h]
        unsigned long pulsePeriodUs = dataInput.getPulsePeriodUs(); // パルス間隔 [us]（ノイズ解析用）
        int currentGear = dataInput.getGear();           // 推定ギア段 (10=1速, 20=2速, ... 60=6速, 0=不明/停止)

        // 加速度の計算 (車速をEMAフィルタしてから微分)
        static float emaSpeed = currentSpeed; // 初期化用
        if (emaSpeed == 0.0f && currentSpeed > 0.0f) emaSpeed = currentSpeed; // 最初の非ゼロで合わせる
        const float ALPHA_SPEED = 0.1f; // 平滑化係数（調整可）
        emaSpeed = ALPHA_SPEED * currentSpeed + (1.0f - ALPHA_SPEED) * emaSpeed;
        
        static float prevEmaSpeed = emaSpeed;
        float acceleration = 0.0f;
        if (dt > 0.0f) {
            acceleration = (emaSpeed - prevEmaSpeed) / dt; // [km/h/s]
        }
        prevEmaSpeed = emaSpeed;
        //
        // 【ペダル開度の出所と単位について】
        //
        // realPedalCAN:
        //   OBD2 PID 0x49 (Accelerator Pedal Position D) をCANで読み取った値。
        //   ECU内部でオフセット補正済みのため、ペダルを離した状態が 0%、全踏みが 100%。
        //   単位: % (0〜100)
        //
        // realPedalADC:
        //   ESP32 GPIO34で実際のペダル電圧を読み取り、開度に換算した値。
        //   0% = キャリブレーション全閉電圧 (IN_MIN_V: GPIOピン上で約0.295V)
        //   100% = キャリブレーション全開電圧 (IN_MAX_V: GPIOピン上で約1.800V)
        //   ※ECUのオフセット補正は行っていないため、アイドル時でも電圧は最小値より高く、
        //     数%〜30%程度の値を示すことがある。realPedalCANとは基準点が異なる。
        //   単位: % (0〜100)
        //
        float realPedalCAN = dataInput.getPedal();
        float realPedalADC = pedalIO.readMainPedal();
        float currentRPM = dataInput.getRPM();        // エンジン回転数 [rpm] CAN(OBD2 PID:0x0C)
        float currentLoad = dataInput.getLoad();      // エンジン負荷 [%] CAN(OBD2 PID:0x04)

        // クラッチ状態の取得
        // true  = 離している（通常走行） / false = 踏んでいる（変速中）
        bool clutchReleased = dataInput.getClutch();

        // クラッチを踏んだ瞬間（Press エッジ）にギアを記録する
        if (prevClutchReleased && !clutchReleased) {
            gearAtClutchPress = currentGear;
        }
        prevClutchReleased = clutchReleased;

        // PID制御量の計算
        // simPedal: PIDコントローラが算出した目標ペダル開度 [%] (0〜30%上限)
        float simPedal = 0.0f;

        if (controlEnabled) {
            if (!clutchReleased) {
                // ── クラッチ踏み込み中 ──
                // RPM低下により「踏んだ時より上位ギア」と判定できた場合のみ制御出力
                // （半クラ状態でギアが噛み始めていると判断）
                if (currentGear > gearAtClutchPress && gearAtClutchPress > 0) {
                    // 上位ギア検出 → 通常PID制御（syncIntegral込み）
                    PIDController savedController = pidController;
                    float calcPedal = pidController.update(currentSpeed, targetSpeed, currentGear, dt, acceleration);
                    if (realPedalADC > calcPedal) {
                        pidController = savedController;
                        pidController.syncIntegral(currentSpeed, targetSpeed, currentGear, realPedalADC);
                        simPedal = pidController.update(currentSpeed, targetSpeed, currentGear, dt, acceleration);
                    } else {
                        simPedal = calcPedal;
                    }
                } else {
                    // ギアが変わっていない or N → 出力0（EMAフィルタだけ更新）
                    simPedal = pidController.update(currentSpeed, targetSpeed, 0, dt, acceleration); // 0.0fが返る
                }
            } else {
                // ── クラッチ離している ──
                if (currentGear == 0) {
                    // ギア不確定（NのままリリースなどでGear:0） → 制御キャンセル
                    // ブレーキ踏んだのと同じ動作（Target車速は保持したまま）
                    controlEnabled = false;
                    pidController.reset(0.0f);
                } else {
                    // 通常のPID制御（syncIntegral込み）
                    PIDController savedController = pidController;
                    float calcPedal = pidController.update(currentSpeed, targetSpeed, currentGear, dt, acceleration);
                    if (realPedalADC > calcPedal) {
                        pidController = savedController;
                        pidController.syncIntegral(currentSpeed, targetSpeed, currentGear, realPedalADC);
                        simPedal = pidController.update(currentSpeed, targetSpeed, currentGear, dt, acceleration);
                    } else {
                        simPedal = calcPedal;
                    }
                }
            }
        } else {
            // 制御OFF時は計算を行わず、出力を0にする
            simPedal = 0.0f;
            // 内部状態をクリア (OLED等のデバッグ表示を0にするため)
            pidController.reset(0.0f);
        }
        
        // simMergePedal: simPedal と realPedalADC のセレクトハイ（大きい方を優先）
        // 制御中でも実際にペダルが深く踏まれた場合はその開度を優先するため
        // 単位: % (0〜100)
        float simMergePedal = (simPedal > realPedalADC) ? simPedal : realPedalADC;

        // 3. ペダル出力の決定とPWM出力
        float outPedalMain = 0.0f; // Mainチャンネル出力開度 [%]
        float outPedalSub  = 0.0f; // Subチャンネル出力開度 [%]
        
        if (controlEnabled) {
            // 制御ON: PIDのセレクトハイ値をPWM出力
            outPedalMain = simMergePedal;
            outPedalSub  = simMergePedal;
        } else {
            // 制御OFF(スルーモード): 実ペダル電圧をそのままPWM出力
            outPedalMain = realPedalADC;
            outPedalSub  = pedalIO.readSubPedal();
        }
        
        pedalIO.setMainPWM(outPedalMain);
        pedalIO.setSubPWM(outPedalSub);


        // ※clutchReleased は上記のPID計算ブロック内で既に取得済み
        
        // 4. シリアル用電圧値および出力予定電圧の算出
        float mainVoltageRaw = pedalIO.readMainPedalVoltage(); // GPIO34の生電圧 (分圧後) [V]
        float subVoltageRaw  = pedalIO.readSubPedalVoltage();  // GPIO35の生電圧 (分圧後) [V]

        // 分圧前の本来の実車ペダル電圧（0.65V〜3.12V）へ復元
        // V1/V2: GPIO上の分圧後電圧を、オペアンプ後の実車電圧スケールへ線形変換した値 [V]
        // 制御OFFのスルーモード時、V1とOutPedalは同じ値になるはず
        float mainVoltage = (mainVoltageRaw - IN_MIN_V) * (3.12f - 0.65f) / (IN_MAX_V - IN_MIN_V) + 0.65f;
        mainVoltage = constrain(mainVoltage, 0.65f, 3.12f);

        float subVoltage = (subVoltageRaw - IN_MIN_V) * (3.12f - 0.65f) / (IN_MAX_V - IN_MIN_V) + 0.65f;
        subVoltage = constrain(subVoltage, 0.65f, 3.12f);

        // OutPedal: PWMで出力する開度[%]から換算した、オペアンプ後の実車電圧相当値 [V] (0.65V〜3.12V)
        float outPedalV = (outPedalMain / 100.0f) * (3.12f - 0.65f) + 0.65f;

        PIDController::PIDDebugInfo pidInfo = pidController.getDebugInfo();

        // 5周期（50ms）に1回シリアルログを送信（間引き）
        static int serialDecimationCounter = 0;
        serialDecimationCounter++;

        if (serialDecimationCounter % 5 == 0) {
            Serial.print("MIN:0,MAX:100,");
            Serial.print("Ctrl:"); Serial.print(controlEnabled ? 100 : 0); Serial.print(",");
            Serial.print("Clutch:"); Serial.print(clutchReleased ? 80 : 0); Serial.print(",");
            Serial.print("Brake:"); Serial.print(brakeActive ? 60 : 0); Serial.print(",");
            Serial.print("TargetSpeed:"); Serial.print(targetSpeed); Serial.print(",");
            Serial.print("CanSpd:"); Serial.print(canSpeedVal, 1); Serial.print(",");
            Serial.print("PulseSpd:"); Serial.print(pulseSpeedVal, 1); Serial.print(",");
            Serial.print("Error:"); Serial.print(targetSpeed > 0.0f ? targetSpeed - canSpeedVal : 0.0f, 1); Serial.print(",");
            Serial.print("P:"); Serial.print(pidInfo.pTerm, 1); Serial.print(",");
            Serial.print("I:"); Serial.print(pidInfo.iTerm, 1); Serial.print(",");
            Serial.print("PWM:"); Serial.print(outPedalMain, 1); Serial.print(",");
            Serial.print("PedalInput:"); Serial.print(realPedalADC, 1); Serial.print(",");
            Serial.print("PulseUs:"); Serial.print(pulsePeriodUs); Serial.print(",");
            Serial.print("Gear:"); Serial.print(currentGear); Serial.print(",");
            // AutoLight
            Serial.print("Bright:"); Serial.print(autoLight.getBrightness(), 1); Serial.print(",");
            Serial.print("DD:"); Serial.print(autoLight.getAccumDistMid(), 1); Serial.print(",");
            Serial.print("BD:"); Serial.print(autoLight.getAccumDistOff(), 1); Serial.print(",");
            Serial.print("LightLogic:"); Serial.print(autoLight.getLogicLightOn() ? 50 : 0); Serial.print(",");
            Serial.print("Accel:"); Serial.print(acceleration, 2); Serial.print(",");
            Serial.print("Logic:"); Serial.print(pidInfo.logicName);
            Serial.println();
        }

        

        // OLED表示更新データのセット（描画は別タスク）
        oled.setValues(
            controlEnabled, targetSpeed, pidController.getInternalTargetSpeed(), canSpeedVal, outPedalMain,
            autoLight.getBrightness(), autoLight.getAccumDistMid(),
            autoLight.getAccumDistOff(), autoLight.getLogicLightOn(),
            acceleration, pidInfo.logicName
        );


    }
}
