#include <iostream>
#include <string>
#include <vector>
#include <fstream> // ifstream, ofstream
#include <sstream> // istringstream

#include "myUtility.h"

const std::string fileName = "../../data/IMU_sample_100Hz.txt";
const int NUM_OF_IMU_DATA = 6;
const double FIRST_TIME = 0.01;
const double IMU_LPF = 0.07; // 出力をエクセルで眺めていい感じの値を選択
const std::vector<double> coordinateModifyFactor{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
const std::vector<double> BIAS{0.264562, -0.1776, -0.30055, -0.02727, 0.010064, 0.009732}; // 静止している0～5sの平均で算出

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


int main(int argc, char** argv)
{
	u<double> input(NUM_OF_IMU_DATA);

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
		// string data -> vector
		input.setUData(buf, coordinateModifyFactor);

		// bias filter
		input.setImuDataBias(BIAS);
		
		// LPF
		input.setLPF(FIRST_TIME, IMU_LPF);
		//input.coutData();

		std::cout << input << std::endl;
	}

	return 0;
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