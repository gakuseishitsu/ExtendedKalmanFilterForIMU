# C++で拡張カルマンフィルタ(EKF)を実装する

## Abstract
* C\+\+の勉強として加速度と角速度の時系列データからカルマンフィルタを使って姿勢を計算するプログラムを書いてみる.
* 具体的なプログラムの流れは以下のものを考えている.

<img src="images/Processing_flow.png" width="700">

## Version
* 2016/07/06 21:00時点 : 立式して満足したところ(コードは書いていない)
* 2016/07/07 18:00時点 : Eigenをダウンロードして使い方を勉強している.
* 2016/07/08 12:00時点 : ひと通り書いてみて, 案の状発散するから初期値をいじってみる.
* 2016/07/09 00:00時点 : クオータニオンの正規化したら発散はしなくなった(値は怪しい). DCMやオイラー角取得関数を書いている.
* 2016/07/10 15:00時点 : 比較用に加速度の値のみからオイラー角を計算する関数を追加. いまだまともな値はでず.
* 2016/07/11 13:00時点 : 共分散行列の初期値をひたすらいじっているが, いまだ発散している. 他に原因ありそう?

## Index
* [How to Sample Data](#how-to-sample-data)
* [Coordinate System](#coordinate-system)
* [Attitude Expression](#attitude-expression)
* [Angular Velocity](#angular-velocity)
* [Gravity Vector in Object coordinate system](#gravity-vector-in-object-coordinate-system)
* [Discretization of Quaternion](#discretization-of-quaternion)
* [Equation of State](#equation-of-state)
* [Jacobian Matrix](#jacobian-matrix)
* [EKF Algorithm and Initial Values](#ekf-algorithm-and-initial-values)
* [Eigen](#eigen)
* [References](#references)

## How to Sample Data
* 加速度と角速度のデータは以下のシステムを組んで取得した. 取得データは「data」に, 取得するのに使ったマイコンのプログラムは「sampling program」に置く.

<img src="images/IMU_sampling_system.png" width="480×480">

## Coordinate System
* 座標系と方向余弦行列ついては以下のように設定する.

<img src="images/Coordinate_system.png" width="700">
<img src="images/DCM.png" width="700">

## Attitude Expression
* DCMとオイラー角の関係, DCMとクオータニオンの関係は以下のように定義する.

<img src="images/Euler.png" width="700">
<img src="images/Quaternion.png" width="700">

## Angular Velocity
* 角速度は物体座標系で定義し, DCM とクオータニオンの時間微分との関係は以下の通りである.

<img src="images/Angular_velocity.png" width="700">

## Gravity Vector in Object coordinate system
* カルマンフィルタで加速度センサの値を出力方程式で用いるためにクオータニオン(状態変数にする予定)と重力加速度の関係を導出しておく.

<img src="images/Gravity_vector.png" width="700">

## Discretization of Quaternion
* カルマンフィルタで状態方程式を立てるためにクオータニオン(状態変数にする予定)を離散化しておく.

<img src="images/Discretization_quaternion.png" width="700">

## Equation of State
* カルマンフィルタを適用させる状態方程式・観測方程式は以下のようになる. 観測方程式が非線形となっているため今回はEKFを使う.

<img src="images/Equation_of_state.png" width="700">

## Jacobian Matrix
* EKFを回すために状態方程式・観測方程式それぞれのヤコビアン行列を求めた.

<img src="images/Jacobian_matrix.png" width="700">

## EKF Algorithm and Initial Values
* EKFのアルゴリズムとその初期値を以下に示す.

<img src="images/EKF.png" width="700">
<img src="images/Initial_values.png" width="700">

## Eigen
* プログラム作成にあたって, Eigenという行列のライブラリを用いた. 忘れないように, 使い方をまとめておく.
```cpp
#include <iostreasm>
#include <Eigen/core> // 基本機能を使うため
#include <Eigen/LU> // 逆行列を使うため
　
int main(){
　
	// Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
	Eigen::Matrix<double, 3, 3> A; //定義
	Eigen::Matrix<double, 3, 3> B;
	Eigen::Matrix<double, 3, 3> C;
	Eigen::Matrix<double, 3, 3> D;
	Eigen::Matrix<double, 3, 3> E;
　
	A << 1.0, 0.0, 0.0, // 代入
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0;
	B.setZero(); // 0で埋める
　
	C = A * B; // 掛け算
	D = A.inverse(); // 逆行列
	E = A.transpose(); // 転置
　
	std::cout << E << std::endl; // シフト演算子も定義されている.
　
	double val = A(0,0); // 要素へのアクセス
	std::cout << val << std::endl; // 1.0
　
	return 0;
}
```

## References
* 座標系の取り方や方向余弦行列DCM, クオータニオンについて以下の資料を参考にした.  
<a href="https://repository.exst.jaxa.jp/dspace/bitstream/a-is/23926/1/naltm00636.pdf">[1] 航空宇宙技術研究所資料 : クオータニオンとオイラー角によるキネマティックス表現の比較について</a>
* 行列のライブラリEigen  
<a href="https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html">[2] Eigen, The Matrix class</a>
* Eigenの基本的な使い方が日本語でまとめられているページ  
<a href="http://blog.livedoor.jp/tek_nishi/archives/8623876.html">[3] でらうま倶楽部, Eigen - C++で使える線形代数ライブラリ</a>