#include "OledDisplay.h"
#include "AutoLight.h"

// コンストラクタ
OledDisplay::OledDisplay()
    : display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, -1),
      _displayMode(MODE_CRUISE_MAIN), _targetInvertUntil(0),
      _ccEnabled(false), _targetSpeed(0), _internalTargetSpeed(0), _currentSpeed(0), _pwmPercent(0),
      _brightness(100.0f), _accumDistMid(0.0f), _accumDistOff(0.0f), _lightOn(false),
      _acceleration(0.0f), _logicName("CTRL_OFF")
{
    mutex = xSemaphoreCreateMutex();
}

void OledDisplay::triggerTargetSpeedInvert() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        _targetInvertUntil = millis() + 400; // 400ms反転表示
        xSemaphoreGive(mutex);
    }
}

void OledDisplay::nextDisplayMode() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        _displayMode = static_cast<DisplayMode>((_displayMode + 1) % MODE_COUNT);
        xSemaphoreGive(mutex);
    }
}

OledDisplay::DisplayMode OledDisplay::getDisplayMode() const {
    return _displayMode;
}

// ─────────────────────────────────────────────────────────────
// init()
// setup() 内で1回呼ぶ。OLED の初期化と起動メッセージを表示する。
// ─────────────────────────────────────────────────────────────
void OledDisplay::init() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
        for (;;);
    }

    // I2C 通信速度を 400kHz に設定
    Wire.setClock(400000);

    // 180度反転: ソフトウェア Y=0 が物理下側、Y=63 が物理上側になる
    // → 黄色エリア(Y=54~63) が物理下側に表示される
    display.setRotation(2);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 28);
    display.print("CC+AutoLight Ready");
    display.display();
}

// ─────────────────────────────────────────────────────────────
// setValues()
// 毎制御周期で呼ぶ。値を保存するだけで描画は行わない（非ブロッキング）。
// ─────────────────────────────────────────────────────────────
void OledDisplay::setValues(bool ccEnabled, float targetSpeed, float internalTargetSpeed, float currentSpeed,
                             float pwmPercent,
                             float brightness, float accumDistMid, float accumDistOff, bool lightOn,
                             float acceleration, const char* logicName)
{
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        _ccEnabled           = ccEnabled;
        _targetSpeed         = targetSpeed;
        _internalTargetSpeed = internalTargetSpeed;
        _currentSpeed        = currentSpeed;
        _pwmPercent          = pwmPercent;
        _brightness          = brightness;
        _accumDistMid        = accumDistMid;
        _accumDistOff        = accumDistOff;
        _lightOn             = lightOn;
        _acceleration        = acceleration;
        _logicName           = logicName;
        xSemaphoreGive(mutex);
    }
}

// ─────────────────────────────────────────────────────────────
// refresh()
// 別タスクから定期的に呼ばれる。保存された値を使って画面を更新する。
// ─────────────────────────────────────────────────────────────
void OledDisplay::refresh() {
    DisplayMode mode;
    bool  ccEnabled;
    float targetSpeed, internalTargetSpeed, currentSpeed, pwmPercent;
    float brightness, accumDistMid, accumDistOff;
    bool  lightOn;
    float acceleration;
    const char* logicName;
    bool  invertTarget;

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        mode                = _displayMode;
        ccEnabled           = _ccEnabled;
        targetSpeed         = _targetSpeed;
        internalTargetSpeed = _internalTargetSpeed;
        currentSpeed        = _currentSpeed;
        pwmPercent          = _pwmPercent;
        brightness          = _brightness;
        accumDistMid        = _accumDistMid;
        accumDistOff        = _accumDistOff;
        lightOn             = _lightOn;
        acceleration        = _acceleration;
        logicName           = _logicName;
        invertTarget        = (millis() < _targetInvertUntil);
        xSemaphoreGive(mutex);
    } else {
        return;
    }

    display.clearDisplay();

    if (mode == MODE_CRUISE_MAIN) {
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // モード①: クルーズコントロール専用画面（デフォルト）
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // 上部 (Y=0~15, 青エリア): CC状態(ON/OFF) と 現在車速 (SPD)
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 4);
        if (ccEnabled) {
            display.fillRect(0, 2, 36, 12, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            display.print(" ON ");
        } else {
            display.drawRect(0, 2, 36, 12, SSD1306_WHITE);
            display.setCursor(4, 4);
            display.print("OFF");
        }
        display.setTextColor(SSD1306_WHITE);

        // 現在車速表示 (右上)
        display.setCursor(48, 4);
        char curBuf[16];
        snprintf(curBuf, sizeof(curBuf), "SPD:%2dkm/h", (int)round(currentSpeed));
        display.print(curBuf);

        // 境界線 Y=15
        display.drawFastHLine(0, 15, 128, SSD1306_WHITE);

        // 中央 (Y=16~47, 青エリア): 目標車速（フォントサイズ3: 超大文字）
        if (targetSpeed > 0.0f) {
            char tgtBuf[10];
            snprintf(tgtBuf, sizeof(tgtBuf), "%3d", (int)round(targetSpeed));

            if (invertTarget) {
                // 白背景・黒文字で反転描画
                display.fillRect(14, 18, 58, 26, SSD1306_WHITE);
                display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            } else {
                display.setTextColor(SSD1306_WHITE);
            }

            display.setTextSize(3);
            display.setCursor(16, 20);
            display.print(tgtBuf);

            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(76, 32);
            display.print("SET");
        } else {
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(3);
            display.setCursor(28, 20);
            display.print("---");

            display.setTextSize(1);
            display.setCursor(84, 32);
            display.print("SET");
        }

        // 下部 (Y=48~63, 黄色エリア): 制御ステータス
        display.setTextSize(1);
        display.setCursor(0, 52);
        if (ccEnabled) {
            char stBuf[24];
            snprintf(stBuf, sizeof(stBuf), "CTRL: %s", logicName);
            display.print(stBuf);
        } else {
            display.print("CTRL: STANDBY");
        }

    } else if (mode == MODE_NORMAL_DETAIL) {
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // モード②: 現状の詳細モニター画面
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // 行1: PWM バーグラフ (Y=0)
        const float pwmDots[] = {25.0f, 50.0f, 75.0f};
        drawProgressBar("PWM", pwmPercent, 0.0f, 100.0f, 0, pwmDots, 3);

        // 境界線 Y=11
        display.drawFastHLine(0, 11, 128, SSD1306_WHITE);

        // 行2: PWM 出力数値表示 + ロジック名表示 (Y=14~45)
        display.setTextColor(SSD1306_WHITE);

        char pwmBuf[16];
        snprintf(pwmBuf, sizeof(pwmBuf), "PWM: %5.1f%%", pwmPercent);
        display.setTextSize(1);
        display.setCursor(4, 14);
        display.print(pwmBuf);

        // 現在動作中の制御ロジック名を表示 (Y=26, テキストサイズ2)
        char logicBuf[20];
        snprintf(logicBuf, sizeof(logicBuf), "%-8s", logicName);
        display.setTextSize(2);
        display.setCursor(4, 26);
        display.print(logicBuf);

        // 黄色エリア (Y=48~63): CC状態 + 車速 (整数) + 内部目標車速 / 設定車速
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        if (ccEnabled) {
            display.fillRect(0, 52, 12, 8, SSD1306_WHITE);
        } else {
            display.drawRect(0, 52, 12, 8, SSD1306_WHITE);
        }

        char buf[20];
        snprintf(buf, sizeof(buf), "%3d>%2d/%2d", (int)round(currentSpeed), (int)internalTargetSpeed, (int)targetSpeed);
        display.setTextSize(2);
        display.setCursor(16, 48);
        display.print(buf);

    } else if (mode == MODE_AUTOLIGHT_DEBUG) {
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // モード③: AutoLight デバッグ画面
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // 1行目 (Y=0): 明るさ (BR) バーグラフ
        drawProgressBar("BR", brightness, 0.0f, 100.0f, 0, NULL, 0);

        // 2行目 (Y=12): 夕暮れ蓄積距離 (DD) バーグラフ (上限500m想定)
        drawProgressBar("DD", accumDistMid, 0.0f, 500.0f, 12, NULL, 0);

        // 3行目 (Y=24): 消灯蓄積距離 (BD) バーグラフ (上限500m想定)
        drawProgressBar("BD", accumDistOff, 0.0f, 500.0f, 24, NULL, 0);

        // 4行目 (Y=36): 各数値テキスト表示
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        char alBuf[32];
        snprintf(alBuf, sizeof(alBuf), "BR:%3d%% DD:%3dm", (int)brightness, (int)accumDistMid);
        display.setCursor(0, 36);
        display.print(alBuf);

        // 下部 (Y=48~63, 黄色エリア): ライト状態 + 車速
        display.setTextSize(1);
        display.setCursor(0, 50);
        if (lightOn) {
            display.fillRect(0, 48, 56, 14, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            display.setCursor(4, 51);
            display.print("LIGHT ON ");
        } else {
            display.drawRect(0, 48, 56, 14, SSD1306_WHITE);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(4, 51);
            display.print("LIGHT OFF");
        }

        display.setTextColor(SSD1306_WHITE);
        char spdAlBuf[16];
        snprintf(spdAlBuf, sizeof(spdAlBuf), "%3dkm/h", (int)round(currentSpeed));
        display.setCursor(64, 51);
        display.print(spdAlBuf);
    }

    display.display();
}


// ─────────────────────────────────────────────────────────────
// drawProgressBar()
// ラベル付きバーグラフを1行描画するヘルパー関数。
// AutoLight1withOLED.ino の実装をベースにしている。
// ─────────────────────────────────────────────────────────────
void OledDisplay::drawProgressBar(const char* label, float val,
                                   float minVal, float maxVal, int y,
                                   const float* dots, int dotCount)
{
    const int TEXT_WIDTH  = 20;             // ラベル幅（3文字 × 6px ≈ 18px + 余白）
    const int GRAPH_X     = TEXT_WIDTH;
    const int GRAPH_WIDTH = 128 - GRAPH_X;  // バー幅 108px

    // ラベル描画
    display.setTextSize(1);
    display.setCursor(0, y);
    display.print(label);

    // 値をクランプ
    if (val < minVal) val = minVal;
    if (val > maxVal) val = maxVal;

    // バー幅の計算
    int barWidth = (int)(((val - minVal) / (maxVal - minVal)) * GRAPH_WIDTH);

    // 目盛り点を y+1 に描画
    int dotY = y + 1;
    for (int i = 0; i < dotCount; i++) {
        if (dots[i] >= minVal && dots[i] <= maxVal) {
            int dotX = GRAPH_X + (int)(((dots[i] - minVal) / (maxVal - minVal)) * (GRAPH_WIDTH - 1));
            display.drawPixel(dotX, dotY, SSD1306_WHITE);
        }
    }

    // バー本体（y+2 から高さ6px）
    int barY      = y + 2;
    int barHeight = 6;
    if (barWidth > 0) {
        display.fillRect(GRAPH_X, barY, barWidth, barHeight, SSD1306_WHITE);
    }
}
