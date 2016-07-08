#include <iostream>
#include <string>
#include <vector>
#include <fstream> // ifstream, ofstream
#include <sstream> // istringstream

#include <Eigen/Core>
#include <Eigen/LU>
#include "myUtility.h"

const std::string fileName = "../../data/IMU_sample_100Hz.txt";
const int NUM_OF_IMU_DATA = 6;
const double FIRST_TIME = 0.01;
const double DT = 0.01;
const double GRAV_ACC = 9.80665;
const double IMU_LPF = 0.07; // èoóÕÇÉGÉNÉZÉãÇ≈í≠ÇﬂÇƒÇ¢Ç¢ä¥Ç∂ÇÃílÇëIë
const std::vector<double> coordinateModifyFactor{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
const std::vector<double> BIAS{0.264562, -0.1776, -0.30055, -0.02727, 0.010064, 0.009732}; // ê√é~ÇµÇƒÇ¢ÇÈ0Å`5sÇÃïΩãœÇ≈éZèo

const double INIT_R = 0.1;
const double INIT_Q = 0.1;
const double INIT_P = 0.01;
const std::vector<double> INIT_X { 0.0, 0.0, 0.0, 1.0 };
const int SIZE_X = 4;
const int SIZE_Y = 3;
const int SIZE_F = SIZE_X;
const int SIZE_H = SIZE_Y;
const int SIZE_G_ROW = SIZE_X; // ROW : çs(â°)
const int SIZE_G_COL = SIZE_Y; // COL : óÒ(èc)
const int SIZE_A_ROW = SIZE_X;
const int SIZE_A_COL = SIZE_X;
const int SIZE_C_ROW = SIZE_Y;
const int SIZE_C_COL = SIZE_X;
const int SIZE_P = SIZE_X;
const int SIZE_Q = SIZE_X;
const int SIZE_R = SIZE_Y;

template<typename T>
class u {
private:
	T t;
	std::vector<T> imuRawData;
	std::vector<T> imuDataBias;
	std::vector<T> imuDataLPF;
public:
	u(const int num);
	~u();

	void setUData(const std::string& s, const std::vector<T>& f);
	void setImuDataBias(const std::vector<T>& b);
	void setLPF(const T ft, const T lpf);

	T getT() const { return t; };
	std::vector<T> getImuDataLPF() const { return imuDataLPF; };
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const u<T>& udata);

template<typename T>
class myKalmanFilter {
private:
	// Eigen::Matrix<T, Rows(â°óÒÇ¢Ç≠Ç¬), Cols(ècóÒÇ¢Ç≠Ç¬)> matrix_name;
	Eigen::Matrix<T, 4, 1> xHat;		// Estimated value of the state valiable
	Eigen::Matrix<T, 4, 1> xHatMinus;	// Advance estimate of the state valiable
	Eigen::Matrix<T, 3, 1> y;			// Observation value
	Eigen::Matrix<T, 4, 1> f;			// System Matrix
	Eigen::Matrix<T, 3, 1> h;			// Transformation matrix from state valiable to observation value
	Eigen::Matrix<T, 4, 4> A;			// Linearized f
	Eigen::Matrix<T, 4, 4> AT;			// Transposition of A
	Eigen::Matrix<T, 4, 3> C;			// Liniarized h
	Eigen::Matrix<T, 3, 4> CT;			// Transposition of C
	Eigen::Matrix<T, 4, 3> g;			// Kalman gain
	Eigen::Matrix<T, 4, 4> P;			// Covariance matrix
	Eigen::Matrix<T, 4, 4> PMinus;		// Advance covariance matrix
	Eigen::Matrix<T, 4, 4> Q;			// Variance matrix of w
	Eigen::Matrix<T, 3, 3> R;			// Variance matrix of v
	Eigen::Matrix<T, 4, 4> I;			// Unit Matrix (4*4) 
public:
	myKalmanFilter(const std::vector<T>& x, const T p, const T q, const T r);
	~myKalmanFilter() {};

	Eigen::Matrix<T, 4, 1> getXHat() const { return xHat; };

	void prediction(const u<T>& input);
	void filter(const u<T>& input);
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const myKalmanFilter<T>& k);

int main(int argc, char** argv)
{
	u<double> input(NUM_OF_IMU_DATA);
	myKalmanFilter<double> kalman(INIT_X, INIT_P, INIT_Q, INIT_R);

	/*
	std::cout << kalman.getXHat() << std::endl;
	while (1) {}
	*/

	std::ifstream file;
	try {
		file.open(fileName, std::ios::in);
		if (!file)
			throw "[INFO] file not exit";
	}
	catch (char* str){
		std::cout << str << std::endl;
		return -1;
	}

	std::string buf;
	// read first line (meaningless header)
	std::getline(file, buf);

	while (std::getline(file, buf))
	{
		// string data -> vector, bias filter, LPF
		input.setUData(buf, coordinateModifyFactor);
		input.setImuDataBias(BIAS);
		input.setLPF(FIRST_TIME, IMU_LPF);

		//std::cout << input << std::endl;

		// kalman filter
		kalman.prediction(input);
		kalman.filter(input);

		std::cout << input.getT() << "," << kalman << std::endl;
	}

	return 0;
}

// constructor of myKalmanFilter
template<typename T>
myKalmanFilter<T>::myKalmanFilter(const std::vector<T>& x, const T p, const T q, const T r) 
{
	xHat << x[0], 
		    x[1], 
		    x[2], 
		    x[3];

	P << p  , 0.0, 0.0, 0.0,
		 0.0, p  , 0.0, 0.0,
		 0.0, 0.0, p  , 0.0,
		 0.0, 0.0, 0.0  , p;

	P << q  , 0.0, 0.0, 0.0,
		 0.0, q  , 0.0, 0.0,
		 0.0, 0.0, q  , 0.0,
		 0.0, 0.0, 0.0, q  ;

	R << r  , 0.0, 0.0,
		 0.0, r  , 0.0,
		 0.0, 0.0, r  ;

	I << 1.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 0.0,
		 0.0, 0.0, 1.0, 0.0,
		 0.0, 0.0, 0.0, 1.0;

	// fill other matrix 0
	xHatMinus.setZero();
	y.setZero();
	f.setZero();
	h.setZero();
	PMinus.setZero();
	g.setZero();
	A.setZero();
	AT.setZero();
	C.setZero();
	CT.setZero();
}

template<typename T>
void myKalmanFilter<T>::prediction(const u<T>& input)
{
	std::vector<T> imu,w,q;
	imu = input.getImuDataLPF();
	w.push_back(static_cast<T>(0));
	for (int i = 3; i < 6; ++i)
		w.push_back(imu[i]); // w[1]~w[3]

	A <<      1.0    ,  0.5*DT*w[3], -0.5*DT*w[2], 0.5*DT*w[1],
		 -0.5*DT*w[3],      1.0    ,  0.5*DT*w[1], 0.5*DT*w[2],
		  0.5*DT*w[2], -0.5*DT*w[1],      1.0    , 0.5*DT*w[3],
		 -0.5*DT*w[1], -0.5*DT*w[2], -0.5*DT*w[3],     1.0    ;

	f = A * xHat;

	xHatMinus = f;

	q.push_back(static_cast<T>(0));
	for (int i = 0; i < 4; ++i)
		q.push_back(xHatMinus[i]); // q[1]~q[4]

	CT <<  2 * q[3] * GRAV_ACC, -2 * q[4] * GRAV_ACC, 2 * q[1] * GRAV_ACC, -2 * q[2] * GRAV_ACC,
		   2 * q[4] * GRAV_ACC,  2 * q[3] * GRAV_ACC, 2 * q[2] * GRAV_ACC,  2 * q[1] * GRAV_ACC,
		  -2 * q[1] * GRAV_ACC, -2 * q[2] * GRAV_ACC, 2 * q[3] * GRAV_ACC,  2 * q[4] * GRAV_ACC;

	PMinus = A * P * A.transpose() + Q;

}

template<typename T>
void myKalmanFilter<T>::filter(const u<T>& input) {
	std::vector<T> imu, a, q;
	imu = input.getImuDataLPF();
	a.push_back(static_cast<T>(0));
	for (int i = 0; i < 3; ++i)
		a.push_back(imu[i]); // a[1]~a[3]
	q.push_back(static_cast<T>(0));
	for (int i = 0; i < 4; ++i)
		q.push_back(xHatMinus[i]); // q[1]~q[4]

	C = CT.transpose();

	Eigen::Matrix<T, 3, 3> TMP;
	TMP = CT * PMinus * C + R;

	g = PMinus * C * TMP.inverse();

	y << a[1],
		 a[2],
		 a[3];

	h << 2 * (q[3] * q[1] - q[2] * q[4]) * GRAV_ACC,
		 2 * (q[2] * q[3] - q[1] * q[4]) * GRAV_ACC,
		 2 * (q[3] * q[3] - q[1] * q[1] - q[2] * q[2] + q[4] * q[4]) * GRAV_ACC;

	xHat = xHatMinus + g * (y - h);



	P = { I - g * CT } *PMinus;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const myKalmanFilter<T>& k) {
	os << k.getXHat()[0] << ","
		<< k.getXHat()[1] << ","
		<< k.getXHat()[2] << ","
		<< k.getXHat()[3];
	return os;
}

// constructor of u
template<typename T>
u<T>::u(const int num)
{
	for (int i = 0; i < num; ++i) {
		imuRawData.push_back(static_cast<T>(0));
		imuDataBias.push_back(static_cast<T>(0));
		imuDataLPF.push_back(static_cast<T>(0));
	}
}

// destructor of u
template<typename T>
u<T>::~u(){}

template<typename T>
void u<T>::setUData(const std::string& s, const std::vector<T>& f) {
	std::vector<std::string> vs = myUtility::split(s, ',');
	t = myUtility::convert_from_string<T>(vs[0]);
	for (int i = 0; i < static_cast<int>(imuRawData.size()); i++)
		imuRawData[i] = f[i] * myUtility::convert_from_string<T>(vs[i+1]);
}

template<typename T>
void u<T>::setImuDataBias(const std::vector<T>& b) {
	for (int i = 0; i < static_cast<int>(imuRawData.size()); i++)
		imuDataBias[i] = imuRawData[i] - b[i];
}

template<typename T>
void u<T>::setLPF(const T ft, const T lpf) {
	if (imuRawData[0] == ft)
		imuDataLPF = imuDataBias;
	else 
		for (int i = 0; i < static_cast<int>(imuRawData.size()); i++)
			imuDataLPF[i] = (lpf)*imuDataBias[i] + (1.0 - lpf) * imuDataLPF[i];
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const u<T>& udata) {
	os << udata.getT() << ","
		<< udata.getImuDataLPF()[0] << ","
		<< udata.getImuDataLPF()[1] << ","
		<< udata.getImuDataLPF()[2] << ","
		<< udata.getImuDataLPF()[3] << ","
		<< udata.getImuDataLPF()[4] << ","
		<< udata.getImuDataLPF()[5];
	return os;
}