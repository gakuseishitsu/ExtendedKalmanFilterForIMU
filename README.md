# 姿勢決定アルゴリズム

## abstract
* C\+\+の勉強として加速度と角速度の時系列データからカルマンフィルタを使って姿勢を計算するプログラムを書いてみる(できるだけC++11の機能を使って).

## Data Sampling
* 加速度と角速度のデータは以下のシステムを組んで取得した. 取得データは「data」に, 取得するのに使ったマイコンのプログラムは「sampling program」に置く.

<img src="images/IMU_sampling_system.png" width="480×480">

## Coordinate System
* 座標系と方向余弦行列ついては以下のように設定する.

<img src="images/Coordinate_system.png" width="700">
<img src="images/DCM.png" width="700">

## Eelationship between DCM and Euler angles and Quaternion
* DCMとオイラー角の関係, DCMとクオータニオンの関係は3-2-1系を用いる.

<img src="images/Euler.png" width="700">
<img src="images/Quaternion.png" width="700">

## 参考文献
* [1] 座標系の取り方や方向余弦行列DCM, クオータニオンについて以下の資料を参考にした.  
<a href="https://repository.exst.jaxa.jp/dspace/bitstream/a-is/23926/1/naltm00636.pdf">航空宇宙技術研究所資料 : クオータニオンとオイラー角によるキネマティックス表現の比較について</a>