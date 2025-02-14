/** \file dab_util_max_preset.cpp
*/

#include "dab_util_max_preset.h"
#include "dab_json_helper.h"
#include "dab_file_io.h"

using namespace dab;
using namespace util;

#pragma mark MaxPreset definition

MaxPreset::MaxPreset(const std::string& pFileName, unsigned int pNr)
	: mFileName(pFileName)
	, mNr(pNr)
{}

const std::map<std::string, std::shared_ptr<AbstractValue>>& 
MaxPreset::namedValues() const
{
	return mNameValueMap;
}

std::shared_ptr<AbstractValue> 
MaxPreset::value(const std::string& pValueName) const throw (Exception)
{
	if (mNameValueMap.find(pValueName) == mNameValueMap.end()) throw dab::Exception("Util Error: max preset does not contain value name " + pValueName, __FILE__, __FUNCTION__, __LINE__);

	return mNameValueMap.at(pValueName);
}

void 
MaxPreset::addNamedValue(const std::string& pValueName, const AbstractValue& pValue) throw (Exception)
{
	if (mNameValueMap.find(pValueName) != mNameValueMap.end()) throw dab::Exception("Util Error: max preset already contains value name " + pValueName, __FILE__, __FUNCTION__, __LINE__);

	mNameValueMap[pValueName] = std::shared_ptr<AbstractValue>(pValue.copy());

	//std::cout << "MaxPreset::addNamedValue " << pValueName << " : " << *(mNameValueMap[pValueName]) << "\n";
}

#pragma mark MaxPresetLoader definition

std::shared_ptr<MaxPreset> 
MaxPresetLoader::loadPreset(const std::string& pFileName, unsigned int pNr) throw (Exception)
{
	try
	{
		std::string restoreString;
		dab::FileIO::get().read(pFileName, restoreString);

		Json::Reader reader;
		Json::Value restoreData;
		dab::JsonHelper& jsonHelper = dab::JsonHelper::get();

		bool parsingSuccessful = reader.parse(restoreString, restoreData);

		if (parsingSuccessful == false) throw dab::Exception("FILE ERROR: failed to parse preset file " + pFileName, __FILE__, __FUNCTION__, __LINE__);

		Json::Value pattrData = jsonHelper.getValue(restoreData, "pattrstorage");
		Json::Value slotsData = jsonHelper.getValue(pattrData, "slots");
		Json::Value slotData = jsonHelper.getValue(slotsData, std::to_string(pNr));
		Json::Value presetData = jsonHelper.getValue(slotData, "data");

		std::shared_ptr<MaxPreset> maxPreset( new MaxPreset(pFileName, pNr) );

		Json::Value::Members memberNames = presetData.getMemberNames();

		for (int mI = 0; mI < memberNames.size(); ++mI)
		{
			//std::cout << "mI " << mI << " : " << memberNames[mI] << "\n";

			std::string presetParameterName = memberNames[mI];
			std::vector<Json::Value> presetParameterValues;
			jsonHelper.getValues(presetData, presetParameterName, presetParameterValues);

			int presetParameterValueCount = presetParameterValues.size();

			if (presetParameterValueCount == 1)
			{
				Json::Value presetParameterValue = presetParameterValues[0];

				//std::cout << "presetParameterValue " << presetParameterValue << " types";
				//std::cout << " int " << presetParameterValue.isInt();
				//std::cout << " double " << presetParameterValue.isDouble();
				//std::cout << " string " << presetParameterValue.isString();
				//std::cout << "\n";

				if (presetParameterValue.isInt()) maxPreset->addNamedValue(presetParameterName, Value<int>(presetParameterValue.asInt()));
				else if (presetParameterValue.isDouble()) maxPreset->addNamedValue(presetParameterName, Value<float>(presetParameterValue.asFloat()));
				else if (presetParameterValue.isString()) maxPreset->addNamedValue(presetParameterName, Value<std::string>(presetParameterValue.asString()));
			}
			else if (presetParameterValueCount > 1)
			{
				// TODO
			}
		}

		return maxPreset;
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("JSON ERROR: failed to load max preset nr " + std::to_string(pNr) + " from file " + pFileName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}
