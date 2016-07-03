# 姿勢決定アルゴリズム

## abstract
* C\+\+の勉強として加速度と角速度の時系列データからカルマンフィルタを使って姿勢を計算するプログラムを書いてみる(できるだけC++11の機能を使って).

## Data Sampling
* 加速度と角速度のデータは以下のシステムを組んで取得した. 取得データは「data」に, 取得するのに使ったマイコンのプログラムは「sampling program」に置く.
![図](IMU_sampling_system.png =480×480)