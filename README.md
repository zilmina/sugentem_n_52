# sugentem_n_52
マイクロマウス2017用マウスsugentem_n_2017のソースコード

---
## 現在判明している不具合
* 走行中にジャイロがずれる
* 走行中に回転運動の制御が発散する
* 斜め走行できない
* 前壁光センサによる角度補正ができない

---
### ハードウェア構成
* 基本仕様
  * 質量：18.15g
  * 全長：52mm
  * 車幅：44mm
  * 全高：20mm
  * トレッド：41mm
* マイコン
  * Renesas electronics製　RX63Nシリーズ　R5F5631NDDFL#V0 x1
* 水晶発振子
  * 村田製作所製　CSTCE10M0G52-R0 x1
* センサー
  * ジャイロセンサー:TDK製　MPU6500 x1
  * フォトダイオード：Lite-On製　LTR-4206E x4
  * エンコーダー：AMS製　AS5047D　x2
* モータードライバ―
  * Texas Instruments製　DRV8836 x1
* モーター
  * Didel製 MK06-10 x2
* 3V3レギュレータ
  * Micrel製　MIC5323-3.3YD5 TR x1
* バッテリー
  * Hyperion製　120mAh x1

---
### ソフトウェア構成
* モード設定
 0. sensor check
 1. search (round trip): 150mm/s
 2. search (only outward way): 150mm/s
 3. try: 150mm/s
 4. try: turn(150mm/s), stright(500mm/s)
 5. try: 300mm/s
 6. try: turn(300mm/s), stright(600mm/s)
 7. try: turn(300mm/s), stright(750mm/s)
 8. 150mm/s stright 90mm x 15区画
 9. counter rotation turn L
 10. 150mm/s turn R
 11. 150mm/s turn L
 12. 300mm/s turn R
 13. 300mm/s turn L
 14. try: turn(300mm/s), stright(2000mm/s)多区間加速
 15. log output
* 制御周期
  * センサー周期：1kHz 
  * モータ出力周期：1kHz 
* センサー値補正について
  * 壁距離センサー
  log近似
  * ジャイロセンサー
  特になし
  * エンコーダー
  特になし
### 迷路アルゴリズム
* 足立法
  * 探索時：区画ごとに方向を逐次演算。ゴールまでの歩数が同じとき、未知区間優先
  * 最短時：区画ごとに方向を逐次演算。ゴールまでの歩数が同じとき、直進優先
### 運動制御アルゴリズム
* 併進（直進）運動
  * エンコーダー速度による併進速度フィードバックPID制御
* 回転運動
  * ジャイロ角速度による回転角速度フィードバックPID制御
