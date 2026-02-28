#include "ImuExtractor.h"

#include "Utils.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace GoProParser
{

using namespace std;


ImuExtractor::ImuExtractor( const std::string &file )
{
    pGPMFStream_ = &gpmf_stream_;

    LOG_INFO << "Opening video file: " << file << std::endl;
    mp4_handle_ = OpenMP4Source( const_cast<char *>( file.c_str() ), MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0 );
    if ( mp4_handle_ == 0 )
    {
        LOG_ERROR << "Could not open video file: " << file << std::endl;
        return;
    }
    metadata_length_ = GetDuration( mp4_handle_ );

    if ( metadata_length_ > 0.0 )
    {
        payload_num_ = GetNumberPayloads( mp4_handle_ );
    }
    uint32_t fr_num, fr_dem;
    frame_count_ = GetVideoFrameRateAndCount( mp4_handle_, &fr_num, &fr_dem );
    frame_rate_ = ( float )fr_num / ( float )fr_dem;

    const uint64_t time_offset = 2082844800;  // 从 1904年1月1日 到 1970年1月1日 的秒数, GoPro的时间戳是从 1904年1月1日 开始的
    movie_creation_time_ = ( uint64_t )( ( getCreationtime( mp4_handle_ ) - time_offset ) * 1000000000 );

    is_ok_ = true;
}

ImuExtractor::~ImuExtractor()
{
    if ( payloadres_ )
    {
        FreePayloadResource( mp4_handle_, payloadres_ );
    }
    if ( pGPMFStream_ )
    {
        GPMF_Free( pGPMFStream_ );
    }

    pPayload_ = NULL;
    CloseSource( mp4_handle_ );
}

void ImuExtractor::getPayloadStamps( uint32_t fourcc,
                                     std::vector<uint64_t> &start_stamps,
                                     uint32_t &total_samples )
{
    start_stamps.clear();
    total_samples = 0;

    for ( uint32_t index = 0; index < payload_num_; index++ )
    {
        GPMF_ERR ret;
        uint32_t payload_size;

        payload_size = GetPayloadSize( mp4_handle_, index );
        payloadres_ = GetPayloadResource( mp4_handle_, payloadres_, payload_size );
        pPayload_ = GetPayload( mp4_handle_, payloadres_, index );

        if ( pPayload_ == NULL )
        {
            cleanUp();
        }
        ret = GPMF_Init( pGPMFStream_, pPayload_, payload_size );
        if ( ret != GPMF_OK )
        {
            cleanUp();
        }

        uint64_t stamp = getStamp( fourcc );  // 当前 payload 的起始时间戳, 单位为 us
        start_stamps.emplace_back( stamp );
    }

    total_samples = getNumOfSamples( fourcc );
}

void ImuExtractor::getImageStamps( vector<uint64_t> &image_stamps )
{
    vector<vector<double>> cam_orient_data;  // N * 4 Camera ORIentation data

    uint64_t curr_stamp = 0, prev_stamp = 0;

    uint64_t total_samples = 0;

    // NOTE: the last payload will be skipped

    for ( uint32_t index = 0; index < payload_num_; index++ )
    {
        GPMF_ERR ret;
        uint32_t payload_size;

        payload_size = GetPayloadSize( mp4_handle_, index );
        payloadres_ = GetPayloadResource( mp4_handle_, payloadres_, payload_size );
        pPayload_ = GetPayload( mp4_handle_, payloadres_, index );

        if ( pPayload_ == NULL )
        {
            cleanUp();
        }
        ret = GPMF_Init( pGPMFStream_, pPayload_, payload_size );
        if ( ret != GPMF_OK )
        {
            cleanUp();
        }

        curr_stamp = getStamp( STR2FOURCC( "CORI" ) );
        curr_stamp = curr_stamp * 1000;  // us to ns

        if ( index > 0 )
        {
            if ( curr_stamp < prev_stamp )
            {
                LOG_ERROR << "previous timestamp should be smaller than current stamp" << endl;
                exit( 1 );
            }

            uint64_t time_span = curr_stamp - prev_stamp;
            double step_size = ( double )time_span / ( double )cam_orient_data.size();

            for ( uint32_t i = 0; i < cam_orient_data.size(); i++ )
            {
                uint64_t stamp = prev_stamp + ( uint64_t )( ( double )i * step_size );
                stamp = movie_creation_time_ + stamp;
                image_stamps.push_back( stamp );
            }
            total_samples += cam_orient_data.size();
        }

        cam_orient_data.clear();
        getScaledData( STR2FOURCC( "CORI" ), cam_orient_data );

        // std::cout << "cam_orient_data num: " << cam_orient_data.size()
        //           << ", size: " << cam_orient_data[0].size() << std::endl;

        prev_stamp = curr_stamp;

        GPMF_Free( pGPMFStream_ );
    }

    LOG_INFO << YELLOW << "The last payload is skipped, it contains ["
             << cam_orient_data.size()  << "] CORI samples" << RESET << std::endl;
}

void ImuExtractor::readImuData( std::deque<AcclMeasurement> &accl_queue,
                                std::deque<GyroMeasurement> &gyro_queue )
{
    vector<vector<double>> accl_data;  // N * 3
    vector<vector<double>> gyro_data;  // N * 3

    uint64_t curr_accl_stamp = 0, prev_accl_stamp = 0;
    uint64_t curr_gyro_stamp = 0, prev_gyro_stamp = 0;

    uint64_t total_samples = 0;

    // NOTE: the last payload will be skipped

    for ( uint32_t index = 0; index < payload_num_; index++ )
    {
        GPMF_ERR ret;
        uint32_t payload_size;

        payload_size = GetPayloadSize( mp4_handle_, index );
        payloadres_ = GetPayloadResource( mp4_handle_, payloadres_, payload_size );
        pPayload_ = GetPayload( mp4_handle_, payloadres_, index );

        if ( pPayload_ == NULL )
        {
            cleanUp();
        }
        ret = GPMF_Init( pGPMFStream_, pPayload_, payload_size );
        if ( ret != GPMF_OK )
        {
            cleanUp();
        }

        curr_accl_stamp = getStamp( STR2FOURCC( "ACCL" ) );
        curr_gyro_stamp = getStamp( STR2FOURCC( "GYRO" ) );

        if ( curr_gyro_stamp != curr_accl_stamp )
        {
            int32_t diff = curr_gyro_stamp - curr_accl_stamp;
            if ( abs( diff ) > 100 )
            {
                LOG_WARNING << "ACCL and GYRO timestamp slightly un-synchronized ...."
                            << ", Index: " << index << " accl stamp: " << curr_accl_stamp
                            << " gyro stamp: " << curr_gyro_stamp << " diff: " << diff << std::endl;
            }
        }

        curr_accl_stamp = curr_accl_stamp * 1000;  // us to ns
        curr_gyro_stamp = curr_gyro_stamp * 1000;  // us to ns

        if ( index > 0 )
        {
            if ( curr_accl_stamp < prev_accl_stamp )
            {
                LOG_ERROR << "accl previous timestamp should be smaller than current stamp" << std::endl;
                exit( 1 );
            }

            if ( curr_gyro_stamp < prev_gyro_stamp )
            {
                LOG_ERROR << "gyro previous timestamp should be smaller than current stamp" << std::endl;
                exit( 1 );
            }

            const uint64_t accl_time_span = curr_accl_stamp - prev_accl_stamp;
            const uint64_t gyro_time_span = curr_gyro_stamp - prev_gyro_stamp;

            const double accl_step_size = ( double )accl_time_span / ( double )accl_data.size();
            const double gyro_step_size = ( double )gyro_time_span / ( double )gyro_data.size();

            for ( uint32_t i = 0; i < accl_data.size(); i++ )
            {
                uint64_t accl_time = prev_accl_stamp + ( uint64_t )( ( double )i * accl_step_size );
                uint64_t accl_stamp = movie_creation_time_ + accl_time;
                vector<double> accl_sample = accl_data.at( i );

                cv::Vec3d accl;
                accl << accl_sample.at( 1 ), accl_sample.at( 2 ), accl_sample.at( 0 );  // Data comes in ZXY order
                accl_queue.push_back( AcclMeasurement( accl_stamp, accl ) );
            }

            for ( uint32_t i = 0; i < gyro_data.size(); ++i )
            {
                uint64_t gyro_time = prev_gyro_stamp + ( uint64_t )( ( double )i * gyro_step_size );
                uint64_t gyro_stamp = movie_creation_time_ + gyro_time;
                vector<double> gyro_sample = gyro_data.at( i );

                cv::Vec3d gyro;
                gyro << gyro_sample.at( 1 ), gyro_sample.at( 2 ), gyro_sample.at( 0 );  // Data comes in ZXY order
                gyro_queue.push_back( GyroMeasurement( gyro_stamp, gyro ) );

                total_samples += 1;
            }
        }

        gyro_data.clear();
        accl_data.clear();
        getScaledData( STR2FOURCC( "ACCL" ), accl_data );
        getScaledData( STR2FOURCC( "GYRO" ), gyro_data );

        if ( accl_data.size() != gyro_data.size() )
        {
            LOG_WARNING << "Index: " << index << ", ACCL and GYRO data are not of same size" << std::endl;
        }

        // std::cout << "accl_data num: " << accl_data.size() << ", size: " << accl_data[0].size()
        //           << ", gyro_data num: " << gyro_data.size() << ", size: " << gyro_data[0].size() << std::endl;

        prev_accl_stamp = curr_accl_stamp;
        prev_gyro_stamp = curr_gyro_stamp;

        GPMF_Free( pGPMFStream_ );
    }

    LOG_INFO << YELLOW << "The last payload is skipped, it contains ["
             << accl_data.size() << "] ACCL samples and ["
             << gyro_data.size() << "] GYRO samples" << RESET << std::endl;
}

void ImuExtractor::printVideoFrameRate()
{
    if ( frame_count_ )
    {
        LOG_INFO << "Frame rate: [" << frame_rate_ << "], num: [" << frame_count_ << "]" << std::endl;
    }
    else
    {
        LOG_WARNING << "Could not get video framerate and frame count" << std::endl;
    }
}

void ImuExtractor::cleanUp()
{
    if ( payloadres_ )
    {
        FreePayloadResource( mp4_handle_, payloadres_ );
    }
    if ( pGPMFStream_ )
    {
        GPMF_Free( pGPMFStream_ );
    }

    pPayload_ = NULL;
    CloseSource( mp4_handle_ );
}

void ImuExtractor::showGpmfStructure()
{
    uint32_t payload_size;
    GPMF_ERR ret = GPMF_OK;

    // Just print the structure of first payload
    // Remaining structure should also be similar
    uint32_t index = 0;
    double in = 0.0, out = 0.0;  // times

    payload_size = GetPayloadSize( mp4_handle_, index );
    payloadres_ = GetPayloadResource( mp4_handle_, payloadres_, payload_size );
    pPayload_ = GetPayload( mp4_handle_, payloadres_, index );

    if ( pPayload_ == NULL )
    {
        cleanUp();
    }

    ret = GetPayloadTime( mp4_handle_, index, &in, &out );
    if ( ret != GPMF_OK )
    {
        cleanUp();
    }

    ret = GPMF_Init( pGPMFStream_, pPayload_, payload_size );
    if ( ret != GPMF_OK )
    {
        cleanUp();
    }

    printf( "PAYLOAD TIME:\n  %.3f to %.3f seconds\n", in, out );
    printf( "GPMF STRUCTURE:\n" );
    // Output (printf) all the contained GPMF data within this payload
    ret = GPMF_Validate( pGPMFStream_, GPMF_RECURSE_LEVELS ); // optional
    if ( GPMF_OK != ret )
    {
        if ( GPMF_ERROR_UNKNOWN_TYPE == ret )
        {
            printf( "Unknown GPMF Type within, ignoring\n" );
            ret = GPMF_OK;
        }
        else
            printf( "Invalid GPMF Structure\n" );
    }

    GPMF_ResetState( pGPMFStream_ );

    GPMF_ERR nextret;
    do
    {
        printf( "  " );
        PrintGPMF( pGPMFStream_ ); // printf current GPMF KLV

        nextret = GPMF_Next( pGPMFStream_, GPMF_RECURSE_LEVELS );

        while (
            nextret ==
            GPMF_ERROR_UNKNOWN_TYPE ) // or just using GPMF_Next(ms, GPMF_RECURSE_LEVELS|GPMF_TOLERANT)
            // to ignore and skip unknown types
            nextret = GPMF_Next( pGPMFStream_, GPMF_RECURSE_LEVELS );

    }
    while ( GPMF_OK == nextret );
    GPMF_ResetState( pGPMFStream_ );
}

GPMF_ERR ImuExtractor::getScaledData( uint32_t fourcc, vector<vector<double>> &readings )
{
    while ( GPMF_OK ==
            GPMF_FindNext( pGPMFStream_,
                           STR2FOURCC( "STRM" ),
                           static_cast<GPMF_LEVELS>( GPMF_RECURSE_LEVELS |
                                   GPMF_TOLERANT ) ) ) // GoPro Hero5/6/7 Accelerometer)
    {
        if ( GPMF_OK !=
                GPMF_FindNext( pGPMFStream_, fourcc, static_cast<GPMF_LEVELS>( GPMF_RECURSE_LEVELS | GPMF_TOLERANT ) ) )
            continue;

        uint32_t samples = GPMF_Repeat( pGPMFStream_ );
        uint32_t elements = GPMF_ElementsInStruct( pGPMFStream_ );
        uint32_t buffersize = samples * elements * sizeof( double );
        double *ptr, *tmpbuffer = ( double * )malloc( buffersize );

        readings.resize( samples );

        if ( tmpbuffer && samples )
        {
            uint32_t i, j;

            // GPMF_FormattedData(ms, tmpbuffer, buffersize, 0, samples); // Output data in LittleEnd, but
            // no scale
            if ( GPMF_OK == GPMF_ScaledData( pGPMFStream_,
                                             tmpbuffer,
                                             buffersize,
                                             0,
                                             samples,
                                             GPMF_TYPE_DOUBLE ) ) // Output scaled data as floats
            {
                ptr = tmpbuffer;
                for ( i = 0; i < samples; i++ )
                {
                    vector<double> sample( elements );
                    for ( j = 0; j < elements; j++ )
                    {
                        sample.at( j ) = *ptr++;
                    }
                    readings.at( i ) = sample;
                }
            }
            free( tmpbuffer );
        }
    }

    GPMF_ResetState( pGPMFStream_ );
    return GPMF_OK;
}

uint64_t ImuExtractor::getStamp( uint32_t fourcc )
{
    GPMF_stream find_stream;

    uint64_t timestamp = 0;
    while ( GPMF_OK ==
            GPMF_FindNext( pGPMFStream_,
                           STR2FOURCC( "STRM" ),
                           static_cast<GPMF_LEVELS>( GPMF_RECURSE_LEVELS |
                                   GPMF_TOLERANT ) ) ) // GoPro Hero5/6/7 Accelerometer)
    {
        if ( GPMF_OK !=
                GPMF_FindNext( pGPMFStream_, fourcc, static_cast<GPMF_LEVELS>( GPMF_RECURSE_LEVELS | GPMF_TOLERANT ) ) )
            continue;

        GPMF_CopyState( pGPMFStream_, &find_stream );
        if ( GPMF_OK == GPMF_FindPrev( &find_stream,
                                       GPMF_KEY_TIME_STAMP,
                                       static_cast<GPMF_LEVELS>( GPMF_CURRENT_LEVEL | GPMF_TOLERANT ) ) )
            timestamp = BYTESWAP64( *( uint64_t * )GPMF_RawData( &find_stream ) );
    }
    GPMF_ResetState( pGPMFStream_ );
    return timestamp;
}

GPMF_ERR ImuExtractor::showCurrentPayload( uint32_t index )
{
    uint32_t payload_size;
    GPMF_ERR ret;

    payload_size = GetPayloadSize( mp4_handle_, index );
    payloadres_ = GetPayloadResource( mp4_handle_, payloadres_, payload_size );
    pPayload_ = GetPayload( mp4_handle_, payloadres_, index );

    if ( pPayload_ == NULL )
    {
        cleanUp();
    }
    ret = GPMF_Init( pGPMFStream_, pPayload_, payload_size );
    if ( ret != GPMF_OK )
    {
        cleanUp();
    }

    GPMF_ERR nextret;
    do
    {
        printf( "  " );
        PrintGPMF( pGPMFStream_ ); // printf current GPMF KLV

        nextret = GPMF_Next( pGPMFStream_, GPMF_RECURSE_LEVELS );

        while (
            nextret ==
            GPMF_ERROR_UNKNOWN_TYPE ) // or just using GPMF_Next(ms, GPMF_RECURSE_LEVELS|GPMF_TOLERANT)
            // to ignore and skip unknown types
            nextret = GPMF_Next( pGPMFStream_, GPMF_RECURSE_LEVELS );

    }
    while ( GPMF_OK == nextret );
    GPMF_ResetState( pGPMFStream_ );

    return GPMF_OK;
}

uint32_t ImuExtractor::getNumOfSamples( uint32_t fourcc )
{
    GPMF_stream find_stream;

    uint32_t total_samples = 0;
    while ( GPMF_OK ==
            GPMF_FindNext( pGPMFStream_,
                           STR2FOURCC( "STRM" ),
                           static_cast<GPMF_LEVELS>( GPMF_RECURSE_LEVELS |
                                   GPMF_TOLERANT ) ) ) // GoPro Hero5/6/7 Accelerometer)
    {
        if ( GPMF_OK !=
                GPMF_FindNext( pGPMFStream_, fourcc, static_cast<GPMF_LEVELS>( GPMF_RECURSE_LEVELS | GPMF_TOLERANT ) ) )
            continue;

        GPMF_CopyState( pGPMFStream_, &find_stream );
        if ( GPMF_OK == GPMF_FindPrev( &find_stream,
                                       GPMF_KEY_TOTAL_SAMPLES,
                                       static_cast<GPMF_LEVELS>( GPMF_CURRENT_LEVEL | GPMF_TOLERANT ) ) )
            total_samples = BYTESWAP32( *( uint32_t * )GPMF_RawData( &find_stream ) );
    }
    GPMF_ResetState( pGPMFStream_ );
    return total_samples;
}

uint64_t ImuExtractor::getPayloadStartStamp( uint32_t fourcc, uint32_t index )
{
    GPMF_ERR ret;
    uint32_t payload_size;

    payload_size = GetPayloadSize( mp4_handle_, index );
    payloadres_ = GetPayloadResource( mp4_handle_, payloadres_, payload_size );
    pPayload_ = GetPayload( mp4_handle_, payloadres_, index );

    if ( pPayload_ == NULL )
    {
        cleanUp();
    }
    ret = GPMF_Init( pGPMFStream_, pPayload_, payload_size );
    if ( ret != GPMF_OK )
    {
        cleanUp();
    }

    uint64_t stamp = getStamp( fourcc );
    return stamp;
}

} // namespace GoProParser