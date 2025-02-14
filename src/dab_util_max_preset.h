/** \file dab_util_max_preset.h
*/

#pragma once

#include "dab_value.h"
#include "dab_singleton.h"
#include <map>

namespace dab
{

namespace util
{

#pragma mark MaxPreset declaration

class MaxPreset
{
public:
	MaxPreset(const std::string& pFileName, unsigned int pNr);

	const std::map<std::string, std::shared_ptr<AbstractValue>>& namedValues() const;
	std::shared_ptr<AbstractValue> value(const std::string& pValueName) const throw (Exception);

	void addNamedValue(const std::string& pValueName, const AbstractValue& pValue) throw (Exception);

protected:
	std::string mFileName;
	unsigned int mNr;

	std::map<std::string, std::shared_ptr<AbstractValue>> mNameValueMap;
};

#pragma mark MaxPresetLoader declaration

class MaxPresetLoader : public Singleton<MaxPresetLoader>
{
public:
	std::shared_ptr<MaxPreset> loadPreset(const std::string& pFileName, unsigned int pNr) throw (Exception);

};

};

};
