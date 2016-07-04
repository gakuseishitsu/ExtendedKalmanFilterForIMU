#include <iostream>
#include <string>
#include <vector>
#include <fstream> // ifstream, ofstream
#include <sstream> // istringstream

const std::string fileName = "../../data/IMU_sample_100Hz.txt";
const int NUM_OF_IMU_DATA = 7;
const std::vector<double> coordinateModifyFactor{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

template<typename T>
class u {
private:
	std::vector<T> imuRawData;
public:
	u();
	~u();
	void setUData(const std::string& s);
	T getImuRawData(int i) const { return imuRawData[i]; }
};

// util func
std::vector<std::string> split(const std::string& input, char delimiter);
template<class T> T convert_from_string(const std::string& str);

int main(int argc, char** argv)
{
	u<double> input;

	std::cout << "reading " << fileName << "..." << std::endl;
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
		input.setUData(buf);
		
		//std::cout << reading_line_buffer << std::endl;
		std::cout << input.getImuRawData(0) << ","
				  << input.getImuRawData(1) << ","
				  << input.getImuRawData(2) << ","
				  << input.getImuRawData(3) << ","
				  << input.getImuRawData(4) << ","
				  << input.getImuRawData(5) << ","
				  << input.getImuRawData(6) << std::endl;
	}

	return 0;
}

// constructor of u
template<typename T>
u<T>::u()
{
	for (int i = 0; i < NUM_OF_IMU_DATA; ++i)
		imuRawData.push_back(static_cast<T>(0));
}

// destructor of u
template<typename T>
u<T>::~u(){}

template<typename T>
void u<T>::setUData(const std::string& s) {
	std::vector<std::string> vs = split(s, ',');
	for (int i = 0; i < imuRawData.size(); i++)
		imuRawData[i] = coordinateModifyFactor[i] * convert_from_string<T>(vs[i]);
}

std::vector<std::string> split(const std::string& input, char delimiter)
{
	std::istringstream stream(input);

	std::string field;
	std::vector<std::string> result;
	while (std::getline(stream, field, delimiter)) {
		result.push_back(field);
	}
	return result;
}

template<class T> 
T convert_from_string(const std::string& str)
{
	std::istringstream stis1(str, std::ios_base::in);
	T c1;
	stis1 >> c1;
	if (stis1.fail() || stis1.bad()) {
		throw "error";
	}
	return c1;
}