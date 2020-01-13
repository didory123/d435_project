#pragma once
#include "stdafx.h"
#include "appconfig-parser/AppConfigParser.h"

AppConfigParser::AppConfigParser()
{

}

AppConfigParser::~AppConfigParser()
{

}
/*

void AppConfigParser::assignValueToField(const std::string& value, const std::string& field)
{
	if (field == CFG_FILE_PATH)
	{
		filePaths_.cfgFilePath = value;
	}
	else if (field == WEIGHTS_FILE_PATH)
	{
		filePaths_.weightsFilePath = value;
	}
	else if (field == NAMES_FILE_PATH)
	{
		filePaths_.namesFilePath = value;
	}aw xssssssssss1
	else
	{
		std::cout << "Unrecognized field name; please check that the appConfig file is configured properly" << std::endl;
	}
}


std::pair<std::string, std::string> AppConfigParser::getFieldAndValueFromLine(const std::string& line)
{
	auto colonIndex = line.find(':');
	
	if (colonIndex == std::string::npos)
	{
		return std::make_pair
			return std::make_pair
	}
	std::string fieldName = line.substr(0, colonIndex + 1);
	
}

bool AppConfigParser::isLineCommented(const std::string& line)
{
	return line[0] == '#';
}

void AppConfigParser::parseAppConfigFieldsByLine(const std::string& filePath)
{
	if (!boost::filesystem::exists(filePath))
	{
		std::cout << "Could not find " << filePath << ", using default values" << std::endl;
		return;
	}
	else
	{
		// Open the specified file
		std::ifstream file(filePath);
		if (file.is_open()) 
		{
			// Read line by line
			std::string line;
			while (getline(file, line)) 
			{
				if (!isLineCommented(line))
				{

				}
			}
			file.close();
		}
	}
}

*/