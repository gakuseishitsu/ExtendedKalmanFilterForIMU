# 姿勢決定アルゴリズム

## Abstract
* C\+\+の勉強として加速度と角速度の時系列データからカルマンフィルタを使って姿勢を計算するプログラムを書いてみる(できるだけC++11の機能を使って).
* 具体的なプログラムの流れは以下のものを考えている.
* 2016/07/06時点 : まだ立式している段階

<img src="images/Processing_flow.png" width="700">

## Data Sampling
* 加速度と角速度のデータは以下のシステムを組んで取得した. 取得データは「data」に, 取得するのに使ったマイコンのプログラムは「sampling program」に置く.

<img src="images/IMU_sampling_system.png" width="480×480">

## Coordinate System
* 座標系と方向余弦行列ついては以下のように設定する.

<img src="images/Coordinate_system.png" width="700">
<img src="images/DCM.png" width="700">

## Relationship between DCM and Euler angles and Quaternion
* DCMとオイラー角の関係, DCMとクオータニオンの関係は以下のように定義する.

<img src="images/Euler.png" width="700">
<img src="images/Quaternion.png" width="700">

## Angular velocity
* 角速度は物体座標系で定義し, DCM とクオータニオンの時間微分との関係は以下の通りである.

<img src="images/Angular_velocity.png" width="700">

## Gravity vector in { b }
* カルマンフィルタで加速度センサの値を出力方程式で用いるためにクオータニオン(状態変数にする予定)と重力加速度の関係を導出しておく.

<img src="images/Gravity_vector.png" width="700">

## 参考文献
* [1] 座標系の取り方や方向余弦行列DCM, クオータニオンについて以下の資料を参考にした.  
<a href="https://repository.exst.jaxa.jp/dspace/bitstream/a-is/23926/1/naltm00636.pdf">航空宇宙技術研究所資料 : クオータニオンとオイラー角によるキネマティックス表現の比較について</a>