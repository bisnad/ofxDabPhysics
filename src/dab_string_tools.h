/** \file dab_string_tools.h
*/

#pragma once

#include "dab_singleton.h"
#include <vector>

namespace dab
{

class StringTools : public Singleton<StringTools>
{
public:
	std::vector<std::string> splitString(const std::string& pString, char pSplitChar, bool pIncludeSplitChar=false);
	std::vector<std::string> wildCardMatch(const std::string& pWildCardString, const std::vector<std::string>& pStrings);

protected:
};

};