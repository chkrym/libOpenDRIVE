#include "AdditionalData.h"

namespace odr
{

UserData::UserData(std::string code, std::string value) : code(code), value(value) {}

DataQualityError::DataQualityError(double xy_absolute, double z_absolute, double xy_relative, double z_relative) :
    xy_absolute(xy_absolute), z_absolute(z_absolute), xy_relative(xy_relative), z_relative(z_relative)
{
}

DataQualityRawData::DataQualityRawData(
    std::string date, std::string source, std::string source_comment, std::string post_processing, std::string post_processing_comment) :
    date(date), source(source), source_comment(source_comment), post_processing(post_processing), post_processing_comment(post_processing_comment)
{
}

bool AdditionalData::empty() const
{
    return this->user_data.empty() && this->includes.empty() && !this->data_quality.has_value();
}

} // namespace odr
