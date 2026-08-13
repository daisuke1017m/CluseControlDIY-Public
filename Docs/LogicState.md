👈 [トップ（README.md）へ戻る](../readme.md)


# クルーズコントロール 制御状態遷移図 (State Diagram)

本ドキュメントでは、クルーズコントロールシステム（ESP32 DIY ASCD）の制御状態の推移および遷移条件を定義します。

---

## 1. 状態遷移図 (Mermaid)

```mermaid
stateDiagram-v2
    [*] --> STANDBY : 起動 (初期状態)

    state "STANDBY\n(制御OFF / 目標車速未設定)" as STANDBY
    state "OFF_READY\n(制御OFF / 目標車速保持)" as OFF_READY
    state "ACTIVE_DRIVING\n(制御ON / 通常走行)" as ACTIVE_DRIVING
    
    state ACTIVE_CLUTCH {
        [*] --> CLUTCH_WAIT : クラッチ踏み込み
        state "シフト待ち / 同ギア保持\n(出力 0%)" as CLUTCH_WAIT
        state "半クラ・シフトアップ検出\n(先行PID制御出力)" as CLUTCH_SHIFTING

        CLUTCH_WAIT --> CLUTCH_SHIFTING : 推定ギア > クラッチ踏み込み時ギア
        CLUTCH_SHIFTING --> CLUTCH_WAIT : ギア変動・N判定
    }

    %% ───────────────────────
    %% 状態遷移ルール
    %% ───────────────────────

    %% 1. STANDBY からの遷移
    STANDBY --> ACTIVE_DRIVING : SET/RESUME押下\n[ブレーキOFF & CAN正常]\n(現在車速でSET)

    %% 2. ACTIVE_DRIVING からの遷移
    ACTIVE_DRIVING --> OFF_READY : ブレーキON または CAN通信エラー
    ACTIVE_DRIVING --> OFF_READY : CANCELスイッチ押下 (設定車速保持)
    ACTIVE_DRIVING --> ACTIVE_CLUTCH : クラッチ踏み込み (Pressエッジ)

    %% 3. ACTIVE_CLUTCH からの遷移
    ACTIVE_CLUTCH --> ACTIVE_DRIVING : クラッチ離す [ギア確定: 1〜6速]
    ACTIVE_CLUTCH --> OFF_READY : クラッチ離す [ニュートラル: Gear 0]
    ACTIVE_CLUTCH --> OFF_READY : ブレーキON または CAN通信エラー
    ACTIVE_CLUTCH --> OFF_READY : CANCELスイッチ押下

    %% 4. OFF_READY からの遷移
    OFF_READY --> ACTIVE_DRIVING : SET/RESUME押下\n[ブレーキOFF & CAN正常]\n(目標車速へRESUME / 現在車速でSET)

    note right of ACTIVE_DRIVING
        【目標車速の変更】
        ・UP/DOWNスイッチ (短押し ±1kph / 長押し ±5kph)
        ・シリアル入力
        ※ACTIVE / OFF_READY のどちらの状態でも更新可能
    end note
```

---

## 2. 主要ステートの説明

| ステート名           | 制御状態 (`controlEnabled`) | 内部状況・動作仕様                                                                                                                   |
| :------------------- | :-------------------------: | :----------------------------------------------------------------------------------------------------------------------------------- |
| **`STANDBY`**        |         **`false`**         | 起動直後の初期状態。目標車速 (`targetSpeed`) が未設定 (`0.0 km/h`) の状態です。                                                      |
| **`OFF_READY`**      |         **`false`**         | ブレーキ、CANCELスイッチ、ニュートラル判定等により制御が一時解除された状態。**目標車速は保持**されており、Resume操作で復帰可能です。 |
| **`ACTIVE_DRIVING`** |         **`true`**          | クルーズコントロール作動中。設定された目標車速に対してPID制御＋各種安全フィードフォワードを行いアクセルPWMを出力します。             |
| **`ACTIVE_CLUTCH`**  |     **`true`** (制限付)     | マニュアル車（MT）のシフト操作中。クラッチが踏み込まれている間の一時状態です。                                                       |

### `ACTIVE_CLUTCH` 内部サブステートの詳細

1. **`CLUTCH_WAIT` (シフト待ち / 出力 0%):**
   - クラッチを切った直後の状態。エンジンの吹き上がりを完全に防止するため、アクセル出力は `0%`（スルー）になります。
2. **`CLUTCH_SHIFTING` (半クラ・シフトアップ検出):**
   - シフト操作中にエンジン回転数低下等によって「切断時より上位のギア」が検出された場合、半クラッチで動力が繋がり始めたと判定し、接続ショックを和らげるため制御出力を先行再開します。
3. **クラッチリリース時の自動判定:**
   - クラッチを離した際、ギアが1〜6速に噛み合っていれば **`ACTIVE_DRIVING`** へ自動復帰します。
   - ニュートラル（Gear 0）のままクラッチを離した場合は空吹かし防止のため自動的に **`OFF_READY`**（制御待機）へ遷移します。
