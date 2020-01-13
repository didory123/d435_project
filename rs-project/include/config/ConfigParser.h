#pragma once
// Abstract function for general config file parsing
class ConfigParser
{
public:
	ConfigParser() {};
	~ConfigParser() {};

	void parseConfigFile(const std::string& configFilePath);

protected:
	virtual void assignFieldFromValue(const std::string& field, const std::string& value) = 0;
	std::pair<std::string, std::string> getFieldAndValueFromLine(const std::string& line);
	bool isLineCommented(const std::string& line);
	bool isFieldHaveValidNumberAsValue(const std::string& field, const std::string& value);
};