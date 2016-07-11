#include <iostream>
#include <string>
#include <vector>
#include <fstream> // ifstream, ofstream
#include <sstream> // istringstream
#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>
#include "myUtility.h"

const std::string fileName = "../../data/IMU_sample_100Hz.txt";
const std::string logFile = "../../data/output.txt";
const int NUM_OF_IMU_DATA = 6;
const double FIRST_TIME = 0.01;
const double DT = 0.01;
const double GRAV_ACC = 9.80665;
const double TO_DEG = 57.2958;
const double IMU_LPF = 0.07; // èoóÕÇÉGÉNÉZÉãÇ≈í≠ÇﬂÇƒÇ¢Ç¢ä¥Ç∂ÇÃílÇëIë
const std::vector<double> coordinateModifyFactor{1.0, -1.0, 1.0, 1.0, -1.0, 1.0}; // MPU6050ÇÃå≥ÇÃç¿ïWånÇ∆é©ï™Ç≈ê›íËÇµÇΩç¿ïWånÇ∆ÇÃí≤êÆ
const std::vector<double> BIAS{0.264562, 0.1776, -0.30055, -0.02727, -0.010064, 0.009732}; // ê√é~ÇµÇƒÇ¢ÇÈ0Å`5sÇÃïΩãœÇ≈éZèo

const std::vector<double> INIT_P{ 0.001, 0.001, 0.001, 0.001 };
const std::vector<double> INIT_Q{ 0.01, 0.01, 0.01, 0.01 };
const std::vector<double> INIT_R{ 0.1, 0.1, 0.1 };
const std::vector<double> INIT_X { 0.0, 0.0, 0.0, 1.0 };
const int SIZE_VECTOR_X = 4;
const int SIZE_VECTOR_Y = 3;

template<typename T>
class u {
private:
	T t;
	std::vector<T> imuRawData;
	std::vector<T> imuDataBias;
	std::vector<T> imuDataLPF;
public:
	u(const int num);
	~u() {};

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
	Eigen::Matrix<T, SIZE_VECTOR_X, 1> xHat;		// Estimated value of the state valiable
	Eigen::Matrix<T, SIZE_VECTOR_X, 1> xHatMinus;	// Advance estimate of the state valiable
	Eigen::Matrix<T, SIZE_VECTOR_Y, 1> y;			// Observation value
	Eigen::Matrix<T, SIZE_VECTOR_X, 1> f;			// System Matrix
	Eigen::Matrix<T, SIZE_VECTOR_Y, 1> h;			// Transformation matrix from state valiable to observation value
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_X> A;			// Linearized f
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_X> AT;			// Transposition of A
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_Y> C;			// Liniarized h
	Eigen::Matrix<T, SIZE_VECTOR_Y, SIZE_VECTOR_X> CT;			// Transposition of C
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_Y> g;			// Kalman gain
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_X> P;			// Covariance matrix
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_X> PMinus;		// Advance covariance matrix
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_X> Q;			// Variance matrix of w
	Eigen::Matrix<T, SIZE_VECTOR_Y, SIZE_VECTOR_Y> R;			// Variance matrix of v
	Eigen::Matrix<T, SIZE_VECTOR_X, SIZE_VECTOR_X> I;			// Unit Matrix
public:
	myKalmanFilter(const std::vector<T>& x, const std::vector<T>& p, const std::vector<T>& q, const std::vector<T>& r);
	~myKalmanFilter() {};

	Eigen::Matrix<T, SIZE_VECTOR_X, 1> getXHat() const { return xHat; };

	void prediction(std::vector<T>& imu);
	void filter(std::vector<T>& imu);

	void normXHat();
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const myKalmanFilter<T>& k);

template<typename T>
Eigen::Matrix<T, 3, 3> getDCM(Eigen::Matrix<T, SIZE_VECTOR_X, 1>& q);

template<typename T>
Eigen::Matrix<T, 3, 1> getEuler(Eigen::Matrix<T, 3, 3>& dcm);

template<typename T>
Eigen::Matrix<T, 3, 1> getEulerFromAccData(std::vector<T>& imu);

int main(int argc, char** argv)
{
	u<double> input(NUM_OF_IMU_DATA);
	myKalmanFilter<double> kalman(INIT_X, INIT_P, INIT_Q, INIT_R);
	std::ifstream file;
	std::ofstream out;

	// open input and output files
	try {
		file.open(fileName, std::ios::in);
		if (!file)
			throw "[INFO] cannot open input file";
		out.open(logFile, std::ios::out);
		if (!out)
			throw "[INFO] cannot open output file";
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
		//std::cout << "LFPdata: " << input << std::endl << std::endl;

		// kalman filter and normalize quaternion
		kalman.prediction(input.getImuDataLPF());
		kalman.filter(input.getImuDataLPF());
		kalman.normXHat();
		std::cout << input.getT() << "," << kalman << std::endl;
		out << input.getT() << "," << kalman << std::endl;

		// get DCM and Euler angles from quaternion
		//Eigen::Matrix<double, 3, 3> DCM = getDCM(kalman.getXHat());
		//Eigen::Matrix<double, 3, 1> E = getEuler(DCM);
		//std::cout << input.getT() << "," << E(0,0)*TO_DEG << "," << E(1, 0)*TO_DEG << "," << E(2, 0)*TO_DEG << std::endl;
		//out << input.getT() << "," << E(0, 0)*TO_DEG << "," << E(1, 0)*TO_DEG << "," << E(2, 0)*TO_DEG << std::endl;

		Eigen::Matrix<double, 3, 1> e = getEulerFromAccData(input.getImuDataLPF());
		//std::cout << input.getT() << "," << e(0,0)*TO_DEG << "," << e(1, 0)*TO_DEG << "," << e(2, 0)*TO_DEG << std::endl;
		//out << input.getT() << "," << e(0, 0)*TO_DEG << "," << e(1, 0)*TO_DEG << "," << e(2, 0)*TO_DEG << std::endl;

		if (input.getT() == 5.0)
			return 0;
	}

	return 0;
}

// constructor of myKalmanFilter
template<typename T>
myKalmanFilter<T>::myKalmanFilter(const std::vector<T>& x, const std::vector<T>& p, const std::vector<T>& q, const std::vector<T>& r)
{
	xHat << x[0], 
		    x[1], 
		    x[2], 
		    x[3];

	P << p[0]  , 0.0, 0.0, 0.0,
		 0.0, p[1]  , 0.0, 0.0,
		 0.0, 0.0, p[2]  , 0.0,
		 0.0, 0.0, 0.0  , p[3];

	P << q[0]  , 0.0, 0.0, 0.0,
		 0.0, q[1]  , 0.0, 0.0,
		 0.0, 0.0, q[2]  , 0.0,
		 0.0, 0.0, 0.0, q[3]  ;

	R << r[0]  , 0.0, 0.0,
		 0.0, r[1]  , 0.0,
		 0.0, 0.0, r[2]  ;

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
void myKalmanFilter<T>::prediction(std::vector<T>& imu)
{
	std::vector<T> w,q;
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
void myKalmanFilter<T>::filter(std::vector<T>& imu) {
	std::vector<T> a, q;
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
		 (q[3] * q[3] - q[1] * q[1] - q[2] * q[2] + q[4] * q[4]) * GRAV_ACC;

	xHat = xHatMinus + g * (y - h);

	P = { I - g * CT } *PMinus;


	//std::cout << "C" << std::endl << C << std::endl << std::endl;
	//std::cout << "xHatMinus" << std::endl << xHatMinus << std::endl << std::endl;
	//std::cout << "PMinus" << std::endl << PMinus << std::endl << std::endl;
	//std::cout << "TMP" << std::endl << TMP << std::endl << std::endl;
	//std::cout << "g" << std::endl << g << std::endl << std::endl;
	//std::cout << "CT" << std::endl << CT << std::endl << std::endl;
	//std::cout << "g*CT" << std::endl << g*CT << std::endl << std::endl;
	//std::cout << "y" << std::endl << y << std::endl << std::endl;
	//std::cout << "h" << std::endl << h << std::endl << std::endl;
	//std::cout << "xHat" << std::endl << xHat << std::endl << std::endl;
	//std::cout << "P" << std::endl << P << std::endl << std::endl;
}


template<typename T>
void myKalmanFilter<T>::normXHat() {
	T norm;
	norm = sqrt(xHat(0, 0)*xHat(0, 0) + xHat(1, 0)*xHat(1, 0) + xHat(2, 0)*xHat(2, 0) + xHat(3, 0)*xHat(3, 0));
	xHat /= norm;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const myKalmanFilter<T>& k) {
	Eigen::Matrix<T, 4, 1> x = k.getXHat();
	os << x(0, 0) << ","
		<< x(1, 0) << ","
		<< x(2, 0) << ","
		<< x(3, 0);
	return os;
}

template<typename T>
Eigen::Matrix<T, 3, 3> getDCM(Eigen::Matrix<T, SIZE_VECTOR_X, 1>& x) {
	Eigen::Matrix<T, 3, 3> dcm;
	std::vector<T> q;
	q.push_back(static_cast<T>(0));
	for (int i = 0; i < 4; ++i)
		q.push_back(x(i,0)); // q[1]~q[4]

	dcm << q[1] * q[1] - q[2] * q[2] - q[3] * q[3] + q[4] * q[4], 2 * (q[1] * q[2] + q[3] * q[4]), 2 * (q[3] * q[1] - q[2] * q[4]),
		2 * (q[1] * q[2] - q[3] * q[4]), q[2] * q[2] - q[3] * q[3] - q[1] * q[1] + q[4] * q[4], 2 * (q[2] * q[3] + q[1] * q[4]),
		2 * (q[3] * q[1] + q[2] * q[4]), 2 * (q[2] * q[4] - q[1] * q[4]), q[3] * q[3] - q[1] * q[1] - q[2] * q[2] + q[4] * q[4];

	return dcm;
}

template<typename T>
Eigen::Matrix<T, 3, 1> getEuler(Eigen::Matrix<T, 3, 3>& dcm) {
	Eigen::Matrix<T, 3, 1> euler;
	T pitch, yaw, roll;

	pitch = asin(-1.0 * dcm(0, 2));
	yaw = atan2(dcm(0, 1) / cos(pitch), dcm(0, 0) / cos(pitch));
	roll = atan2(dcm(1, 2) / cos(pitch), dcm(2, 2) / cos(pitch));

	euler << roll,
		pitch,
		yaw;

	return euler;
}

template<typename T>
Eigen::Matrix<T, 3, 1> getEulerFromAccData(std::vector<T>& imu) {
	Eigen::Matrix<T, 3, 1> euler;

	T roll, pitch, yaw, acc;

	acc = sqrt(imu[0] * imu[0] + imu[1] * imu[1] + imu[2] * imu[2]);

	pitch = asin(-1.0 * imu[0] / acc);
	yaw = static_cast<T>(0);
	roll = atan2(imu[1] / acc / cos(pitch), imu[2] / acc / cos(pitch));

	euler << roll,
		pitch,
		yaw;

	return euler;
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
	if (t == ft)
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