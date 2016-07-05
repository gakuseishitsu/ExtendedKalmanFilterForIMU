#include <vector>
#include <fstream> // ifstream, ofstream
#include <sstream> // istringstream


namespace myUtility {
	std::vector<std::string> split(const std::string& input, char delimiter);
	template<class T> T convert_from_string(const std::string& str);
}

std::vector<std::string> myUtility::split(const std::string& input, char delimiter)
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
T myUtility::convert_from_string(const std::string& str)
{
	std::istringstream stis1(str, std::ios_base::in);
	T c1;
	stis1 >> c1;
	if (stis1.fail() || stis1.bad()) {
		throw "error";
	}
	return c1;
}
