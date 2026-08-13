#include "AutoLight.h"

// ─────────────────────────────────────────────────────────────
// コンストラクタ
// ─────────────────────────────────────────────────────────────
AutoLight::AutoLight()
    : _startupChecked(false),
      _logicLightOn(false),
      _lightOnTimestamp(0),
      _canStarted(false),
      _canStartTimestamp(0),
      _accumDistLow(0.0f),
      _accumDistMid(0.0f),
      _accumDistOff(0.0f),
      _accumDistOffTunnel(0.0f),
      _lowBrightnessStart(0),
      _normalizedBrightness(100.0f)
{
}

// ─────────────────────────────────────────────────────────────
// init()
// setup() 内で1回呼ぶ。ピン設定と ADC アッテネーションを設定する。
// ─────────────────────────────────────────────────────────────
void AutoLight::init() {
    // 出力ピン: Active High (LOW=消灯でデフォルト)
    pinMode(AUTOLIGHT_OUTPUT_PIN, OUTPUT);
    digitalWrite(AUTOLIGHT_OUTPUT_PIN, LOW);

    // GPIO36 (VP) 専用で ADC_11db を設定
    // ペダルADC (GPIO34/35) は PedalIO で設定済みの ADC_6db を維持する
    analogSetPinAttenuation(AUTOLIGHT_SENSOR_PIN, ADC_11db);
}

// ─────────────────────────────────────────────────────────────
// update()
// 毎制御周期で呼ぶ。CAN車速と経過時間から走行距離を計算し、
// AutoLight1withOLED のロジックをそのまま踏襲して点灯制御を行う。
// ─────────────────────────────────────────────────────────────
void AutoLight::update(float speedKmh, bool canActive, float dt) {
    uint32_t now = millis();

    // 1. 明るさ読み取り (GPIO36、20回平均、0〜100% に正規化)
    long sum = 0;
    for (int i = 0; i < 20; i++) {
        sum += analogRead(AUTOLIGHT_SENSOR_PIN);
    }
    _normalizedBrightness = (sum / 20.0f / AL_MAX_ADC_VALUE) * 100.0f;

    // 2. 起動時の即点灯判定
    // CAN通信が初めて有効になってから AL_STARTUP_DELAY_MS の間は「猶予期間」として
    // 継続的に明るさを監視し、閾値(_THRESHOLD_DUSK_INIT)を下回った瞬間に即点灯させる。
    if (!_startupChecked) {
        if (canActive && !_canStarted) {
            _canStarted = true;
            _canStartTimestamp = now;
        }

        if (_canStarted) {
            if (_normalizedBrightness <= AL_THRESHOLD_DUSK_INIT) {
                // 猶予期間中に暗くなったら即点灯＆通常モードへ
                _logicLightOn = true;
                _lightOnTimestamp = now;
                _startupChecked = true;
            } else if (now - _canStartTimestamp >= (uint32_t)AL_STARTUP_DELAY_MS) {
                // 時間切れ: 明るいまま猶予期間を過ぎたら通常モードへ
                _startupChecked = true;
            }
        }
        
        // 通常モード(_startupChecked==true)に移行するまでは以後の処理を行わない
        if (!_startupChecked) {
            digitalWrite(AUTOLIGHT_OUTPUT_PIN, _logicLightOn ? HIGH : LOW);
            return;
        }
    }

    // CAN通信が成立していない場合は、無条件にライトOFFにして状態をリセットする
    if (!canActive) {
        _logicLightOn = false;
        _accumDistLow = 0;
        _accumDistMid = 0;
        _accumDistOff = 0;
        _accumDistOffTunnel = 0;
        _lowBrightnessStart = 0;
        digitalWrite(AUTOLIGHT_OUTPUT_PIN, LOW);
        return;
    }


    // 3. 距離ベースロジック
    //    車速[km/h] → [m/s] に変換して dt[s] を掛けて移動距離[m]を求める
    float moveDist = (speedKmh / 3.6f) * dt;

    if (!_logicLightOn) {
        // ── 消灯中 ──────────────────────────────────────────────

        // ND (トンネル): AL_THRESHOLD_TUNNEL 以下が AL_DIST_TUNNEL 続いたら点灯
        //               または低速かつ AL_TIME_LOW_SPEED_TRIGGER ms 続いたら点灯
        if (_normalizedBrightness <= AL_THRESHOLD_TUNNEL) {
            _accumDistLow += moveDist;
            if (_lowBrightnessStart == 0) _lowBrightnessStart = now;
            if (_accumDistLow >= AL_DIST_TUNNEL ||
                (speedKmh <= AL_LOW_SPEED_KPH &&
                 (now - _lowBrightnessStart >= (uint32_t)AL_TIME_LOW_SPEED_TRIGGER))) {
                _logicLightOn = true;
                _lightOnTimestamp = now;
            }
        } else {
            _accumDistLow = 0;
            _lowBrightnessStart = 0;
        }

        // DD (夕暮れ): AL_THRESHOLD_DUSK 以下が AL_DIST_DUSK 続いたら点灯
        if (_normalizedBrightness <= AL_THRESHOLD_DUSK) {
            _accumDistMid += moveDist;
            if (_accumDistMid >= AL_DIST_DUSK) {
                _logicLightOn = true;
                _lightOnTimestamp = now;
            }
        } else {
            _accumDistMid = 0;
        }

        // 消灯中は BD/BDトンネル をリセット
        _accumDistOff        = 0;
        _accumDistOffTunnel  = 0;

    } else {
        // ── 点灯中 ──────────────────────────────────────────────
        bool canOff = (now - _lightOnTimestamp >= (uint32_t)AL_MIN_ON_TIME_MS);

        // BD (通常消灯): AL_THRESHOLD_OFF 超えが AL_DIST_OFF 続いたら消灯
        if (_normalizedBrightness > AL_THRESHOLD_OFF) {
            _accumDistOff += moveDist;
        } else {
            _accumDistOff = 0;
        }

        // BD_T (トンネル出口消灯): AL_THRESHOLD_OFF_TUNNEL 超えが AL_DIST_OFF_TUNNEL 続いたら消灯
        if (_normalizedBrightness > AL_THRESHOLD_OFF_TUNNEL) {
            _accumDistOffTunnel += moveDist;
        } else {
            _accumDistOffTunnel = 0;
        }

        if (canOff) {
            if (_accumDistOff >= AL_DIST_OFF || _accumDistOffTunnel >= AL_DIST_OFF_TUNNEL) {
                _logicLightOn = false;
                _accumDistLow = 0;
                _accumDistMid = 0;
            }
        }
    }

    // 4. GPIO15 出力 (Active High)
    //    物理SWは出力線上に配置されているため、ソフトウェア側のSW判定は不要
    digitalWrite(AUTOLIGHT_OUTPUT_PIN, _logicLightOn ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────
// ゲッター
// ─────────────────────────────────────────────────────────────
bool  AutoLight::isLightOn()       const { return _logicLightOn; }
float AutoLight::getBrightness()   const { return _normalizedBrightness; }
float AutoLight::getAccumDistMid() const { return _accumDistMid; }
float AutoLight::getAccumDistOff() const { return _accumDistOff; }
bool  AutoLight::getLogicLightOn() const { return _logicLightOn; }
