#pragma once

#include <string>
#include <vector>
#include <deque>

#include <opencv2/core/types.hpp>

#include "GPMF_parser.h"
#include "GPMF_mp4reader.h"

namespace GoProParser
{

struct AcclMeasurement
{
    AcclMeasurement() = default;

    AcclMeasurement( const uint64_t &_timestamp, const cv::Vec3d &_data )
        : timestamp( _timestamp ), data( _data ) {}

    uint64_t timestamp;
    cv::Vec3d data;
};

struct GyroMeasurement
{
    GyroMeasurement() = default;

    GyroMeasurement( const uint64_t &_timestamp, const cv::Vec3d &_data )
        : timestamp( _timestamp ), data( _data ) {}

    uint64_t timestamp;
    cv::Vec3d data;
};

class ImuExtractor
{
private:
    bool is_ok_ = false;

    GPMF_stream gpmf_stream_;
    GPMF_stream *pGPMFStream_;
    double metadata_length_;
    size_t mp4_handle_;
    uint32_t payload_num_;
    uint32_t *pPayload_ = NULL;
    size_t payloadres_ = 0;

    uint32_t frame_count_;
    float frame_rate_;
    uint64_t movie_creation_time_;

public:
    ImuExtractor( const std::string &file );
    ~ImuExtractor();

    bool isOk() const
    {
        return is_ok_;
    }

    void printVideoFrameRate();

    uint32_t getImageCount() const
    {
        return frame_count_;
    }

    uint64_t getVideoCreationTime() const
    {
        return movie_creation_time_;
    }

    void getPayloadStamps( uint32_t fourcc,
                           std::vector<uint64_t> &start_stamps,
                           uint32_t &total_samples );

    void getImageStamps( std::vector<uint64_t> &image_stamps );

    void readImuData( std::deque<AcclMeasurement> &accl_data,
                      std::deque<GyroMeasurement> &gyro_data );

private:

    void cleanUp();

    void showGpmfStructure();

    GPMF_ERR getScaledData( uint32_t fourcc, std::vector<std::vector<double>> &readings );

    uint64_t getStamp( uint32_t fourcc );

    uint32_t getNumOfSamples( uint32_t fourcc );

    GPMF_ERR showCurrentPayload( uint32_t index );

    uint64_t getPayloadStartStamp( uint32_t fourcc, uint32_t index );
};

} // namespace GoProParser