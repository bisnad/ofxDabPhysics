/** \file dab_string_tools.cpp
*/

#include "dab_string_tools.h"
#include <string>

using namespace dab;

std::vector<std::string> 
StringTools::splitString(const std::string& pString, char pSplitChar, bool pIncludeSplitChar)
{
	std::string splitChar(1, pSplitChar);
	int stringLength = pString.length();
	std::vector<std::string> _splitString;
	std::string::size_type prev_pos = 0, pos = 0;

	while ((pos = pString.find(splitChar, pos)) != std::string::npos)
	{
		if (prev_pos < pos)
		{
			std::string substring(pString.substr(prev_pos, pos - prev_pos));
			_splitString.push_back(substring);
		}

		if (pIncludeSplitChar) _splitString.push_back(splitChar);

		prev_pos = ++pos;
	}

	if (prev_pos < stringLength)
	{
		std::string substring(pString.substr(prev_pos, stringLength - prev_pos));
		_splitString.push_back(substring);
	}

	return _splitString;
}

std::vector<std::string> 
StringTools::wildCardMatch(const std::string& pWildCardString, const std::vector<std::string>& pStrings)
{
	char wildCardCharacter = '*';
	std::string wildCardString(1, wildCardCharacter);
	std::vector<std::string> splitStrings = splitString(pWildCardString, wildCardCharacter, true);

	std::cout << "split wild card string:\n";
	for (int i = 0; i < splitStrings.size(); ++i)
	{
		std::cout << "i " << i << " : " << splitStrings[i] << "\n";
	}

	int splitCount = splitStrings.size();

	// trivial case: splitCount = 1
	if (splitCount == 1 && splitStrings[0] == wildCardString) return pStrings;

	int stringCount = pStrings.size();
	std::vector<std::string> matchingStrings;

	for (int sI = 0; sI < stringCount; ++sI)
	{
		std::string string = pStrings[sI];
		bool flexibleMatch = false;
		std::string::size_type matchPos;

		std::cout << ">>>>>> test string " << string << " <<<<<<<\n";

		for (int ssI = 0; ssI < splitCount; ++ssI)
		{
			const std::string& splitString = splitStrings[ssI];

			std::cout << ">>> iter " << ssI << "<<<\n";
			std::cout << "splitString: " << splitString << "\n";
			std::cout << "testString: " << string << "\n";

			if (splitString == wildCardString)
			{
				flexibleMatch = true;
				continue;
			}

			matchPos = string.find(splitString);

			if (matchPos == std::string::npos)
			{
				std::cout << "splitString: " << splitString << " not found\n";
				break;
			}
			else if (flexibleMatch == false && matchPos > 0)
			{
				std::cout << "splitString: " << splitString << " found but not at the beginning\n";
				break;
			}
			else
			{
				std::cout << "splitString: " << splitString << " found\n";
				string = string.substr(matchPos + splitString.length(), string.length() - (matchPos + splitString.length()));
			}
		}

		std::cout << "remaining string " << string << "\n";

		if (string.length() > 0 && splitStrings[splitCount - 1] != wildCardString)
		{
			std::cout << "remaining string not empty\n";
			break;
		}

		std::cout << "full string match\n";

		matchingStrings.push_back(pStrings[sI]);

	}

	return matchingStrings;
}