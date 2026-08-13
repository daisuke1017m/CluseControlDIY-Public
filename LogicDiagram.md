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
    STANDBY --> ACTIVE_DRIVING : SET/RESUME押下\n[ブレーキOFF & CAN正常]\n(現在車速でSET: min 30kph)

    %% 2. ACTIVE_DRIVING からの遷移
    ACTIVE_DRIVING --> OFF_READY : ブレーキON または CAN通信エラー
    ACTIVE_DRIVING --> STANDBY : CANCELスイッチ押下
    ACTIVE_DRIVING --> ACTIVE_CLUTCH : クラッチ踏み込み (Pressエッジ)

    %% 3. ACTIVE_CLUTCH からの遷移
    ACTIVE_CLUTCH --> ACTIVE_DRIVING : クラッチ離す [ギア確定: 1〜6速]
    ACTIVE_CLUTCH --> OFF_READY : クラッチ離す [ニュートラル: Gear 0]
    ACTIVE_CLUTCH --> OFF_READY : ブレーキON または CAN通信エラー
    ACTIVE_CLUTCH --> STANDBY : CANCELスイッチ押下

    %% 4. OFF_READY からの遷移
    OFF_READY --> ACTIVE_DRIVING : SET/RESUME押下\n[ブレーキOFF & CAN正常]\n(目標車速へRESUME)
    OFF_READY --> STANDBY : CANCELスイッチ押下
    OFF_READY --> STANDBY : 車速 ≦ 目標車速 × 50%\n(大幅減速による自動リセット)

    %% 車速変更ノード (自己遷移的注記)
    note right of ACTIVE_DRIVING
        【目標車速の変更】
        ・UP/DOWNスイッチ (短押し/長押し)
        ・シリアル入力
        ※ACTIVE / OFF_READY のどちらでも更新可能
    end note
