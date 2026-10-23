#pragma once
#include <optional>
#include <string>
#include <vector>

namespace odr
{

// OpenDRIVE 1.6.1, 5.1 User Data
struct UserData
{
    UserData(std::string code, std::string value);

    std::string code = "";
    std::string value = "";
    std::string content = ""; // serialized child XML content, if any
};

// OpenDRIVE 1.6.1, 5.4 Description of the data quality: absolute or relative errors of road data
struct DataQualityError
{
    DataQualityError(double xy_absolute, double z_absolute, double xy_relative, double z_relative);

    double xy_absolute = 0;
    double z_absolute = 0;
    double xy_relative = 0;
    double z_relative = 0;
};

// OpenDRIVE 1.6.1, 5.4 Description of the data quality: metadata of the raw data
struct DataQualityRawData
{
    DataQualityRawData(
        std::string date, std::string source, std::string source_comment, std::string post_processing, std::string post_processing_comment);

    std::string date = "";
    std::string source = ""; // "sensor", "cadaster" or "custom"
    std::string source_comment = "";
    std::string post_processing = ""; // "raw", "cleaned", "processed" or "fused"
    std::string post_processing_comment = "";
};

struct DataQuality
{
    DataQuality() = default;

    std::optional<DataQualityError>   error;
    std::optional<DataQualityRawData> raw_data;
};

// OpenDRIVE 1.6.1, 5 Additional Data: <userData>, <include> and <dataQuality> may be stored at any element
struct AdditionalData
{
    AdditionalData() = default;

    bool empty() const;

    std::vector<UserData>      user_data;
    std::vector<std::string>   includes; // @file of <include> elements
    std::optional<DataQuality> data_quality;
};

} // namespace odr
