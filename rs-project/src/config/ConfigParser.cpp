#pragma once
#include "stdafx.h"
#include "config\ConfigParser.h"

void ConfigParser::parseConfigFile(const std::string & configFilePath)
{
	if (!boost::filesystem::exists(configFilePath))
	{
		std::string exceptionMessage = "The file " + configFilePath + " was not found! Please ensure that the file exists.";
		throw std::runtime_error(exceptionMessage);
	}
	else
	{
		// Open the specified file
		std::ifstream file(configFilePath);
		if (file.is_open())
		{
			// Read line by line
			std::string line;
			while (getline(file, line))
			{
				// skip empty lines
				while (line.length() == 0)
				{
					getline(file, line);
				}
				if (!isLineCommented(line))
				{
					auto fieldAndValuePair = getFieldAndValueFromLine(line);
					assignFieldFromValue(fieldAndValuePair.first, fieldAndValuePair.second);
				}
			}
			file.close();
		}
	}
}

std::pair<std::string, std::string> ConfigParser::getFieldAndValueFromLine(const std::string & line)
{
	auto colonIndex = line.find(':');

	if (colonIndex == std::string::npos)
	{
		return std::make_pair(line, "");
	}
	std::string fieldName = line.substr(0, colonIndex);
	std::string value = line.substr(colonIndex + 1);

	return std::make_pair(fieldName, value);
}

bool ConfigParser::isLineCommented(const std::string & line)
{
	return line[0] == '#';
}

// C-style helper function to check if given string is a valid number
// Prints a warning is the value is not a number
// Taken from: https://stackoverflow.com/questions/29169153/how-do-i-verify-a-string-is-valid-double-even-if-it-has-a-point-in-it
bool ConfigParser::isFieldHaveValidNumberAsValue(const std::string & field, const std::string & value)
{
	char* end = 0;
	double val = strtod(value.c_str(), &end);
	bool isStringNumber = end != value.c_str() && *end == '\0' && val != HUGE_VAL;

	if (!isStringNumber)
	{
		std::cout << "WARNING: The field " + field + " does not have a valid number as value. Given value is " + value + ". Using default value instead" << std::endl;
	}
	return isStringNumber;
}