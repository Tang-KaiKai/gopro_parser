#include "Utils.h"
#include "ImuExtractor.h"
#include "VideoExtractor.h"

#include <iostream>
#include <fstream>
#include <filesystem>

using namespace std;
using namespace GoProParser;

int main( int argc, char *argv[] )
{
    /// 从命令行参数读取参数
    if ( Tool::ParseCmdLine( argc, argv, "--h" ) || argc < 3 )
    {
        cout << "Usage: ./gopro_to_euroc\n"
             << "    --g             : the path to the GoPro video file\n"
             << "    --d             : the directory to save the extracted data\n"
             << "    --s             : the scale to resize the image, default: 1.0\n"
             << "    --gray          : whether to save grayscale images, default: false\n"
             << "    --display       : whether to display images during processing, default: false\n"
             << "    --ignore_img    : whether to ignore images during processing, default: false\n"
             << "    --ignore_imu    : whether to ignore IMU data during processing, default: false\n"
             << "    --h             : show this help message" << endl;

        return 0;
    }

    string gopro_video_path;
    if ( !Tool::ParseCmdLine( argc, argv, "--g", &gopro_video_path ) )
    {
        LOG_ERROR << "Please provide the GoPro video path using --g";
        return -1;
    }
    LOG_INFO << "GoPro video path: " << gopro_video_path << endl;

    string data_save_dir;
    if ( !Tool::ParseCmdLine( argc, argv, "--d", &data_save_dir ) )
    {
        LOG_ERROR << "Please provide the data save directory using --d";
        return -1;
    }
    LOG_INFO << "Data save directory: " << data_save_dir << endl;

    float scale = 1.0;
    {
        string value;
        if ( Tool::ParseCmdLine( argc, argv, "--s", &value ) )
        {
            try
            {
                scale = std::stof( value );
            }
            catch ( const std::exception &e )
            {
                LOG_ERROR << "Invalid image scale value: " << value << ". Using default scale of 1.0";
                scale = 1.0;
            }
        }
    }
    LOG_INFO << "Image saved scale: " << scale << endl;

    const bool use_grayscale = Tool::ParseCmdLine( argc, argv, "--gray" );
    LOG_INFO << "Save grayscale images: " << ( use_grayscale ? "true" : "false" ) << endl;

    const bool display_images = Tool::ParseCmdLine( argc, argv, "--display" );
    LOG_INFO << "Display images during processing: " << ( display_images ? "true" : "false" ) << endl;

    const bool ignore_images = Tool::ParseCmdLine( argc, argv, "--ignore_img" );
    LOG_INFO << "Ignore images during processing: " << ( ignore_images ? "true" : "false" ) << endl;

    const bool ignore_imu = Tool::ParseCmdLine( argc, argv, "--ignore_imu" );
    LOG_INFO << "Ignore IMU data during processing: " << ( ignore_imu ? "true" : "false" ) << endl;

    if ( ignore_images && ignore_imu )
    {
        LOG_WARNING << "Both images and IMU data are set to be ignored. Nothing to process." << endl;
        return 0;
    }

    cout << endl;

    /// 解析 IMU 数据

    ImuExtractor imu_extractor( gopro_video_path );
    if ( !imu_extractor.isOk() )
    {
        return 0;
    }
    {
        vector<uint64_t> start_stamps;
        uint32_t total_samples = 0;

        imu_extractor.getPayloadStamps( STR2FOURCC( "ACCL" ), start_stamps, total_samples );
        LOG_INFO << "[ACCL] Payload Number: " << start_stamps.size()
                 << ", Start stamp( us ): " << start_stamps[0]
                 << ", End stamp( us ): " << start_stamps.back()
                 << ", Total Samples: " << total_samples << endl;

        imu_extractor.getPayloadStamps( STR2FOURCC( "GYRO" ), start_stamps, total_samples );
        LOG_INFO << "[GYRO] Payload Number: " << start_stamps.size()
                 << ", Start stamp( us ): " << start_stamps[0]
                 << ", End stamp( us ): " << start_stamps.back()
                 << ", Total Samples: " << total_samples << endl;

        imu_extractor.getPayloadStamps( STR2FOURCC( "CORI" ), start_stamps, total_samples );
        LOG_INFO << "[CORI] Payload Number: " << start_stamps.size()
                 << ", Start stamp( us ): " << start_stamps[0]
                 << ", End stamp( us ): " << start_stamps.back()
                 << ", Total Samples: " << total_samples << endl;
    }

    std::deque<AcclMeasurement> accl_queue;
    std::deque<GyroMeasurement> gyro_queue;
    imu_extractor.readImuData( accl_queue, gyro_queue );
    LOG_INFO << "accl measurements num: " << accl_queue.size() << ", gyro measurements num: " << gyro_queue.size() << endl;

    // 保存 IMU 数据
    if ( !ignore_imu )
    {
        const std::string imu_folder = data_save_dir + "/imu0";
        if ( !std::filesystem::is_directory( imu_folder ) )
        {
            std::filesystem::create_directories( imu_folder );
        }
        const std::string imu_file = imu_folder + "/data.csv";

        ofstream stream;
        stream.open( imu_file );
        stream << std::fixed << std::setprecision( 19 );
        stream << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
               "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
               << endl;

        while ( !accl_queue.empty() && !gyro_queue.empty() )
        {
            AcclMeasurement accl = accl_queue.front();
            GyroMeasurement gyro = gyro_queue.front();

            uint64_t stamp;
            int64_t diff = accl.timestamp - gyro.timestamp;
            if ( abs( diff ) > 100000 )
            {
                LOG_WARNING << diff << " ns difference between gyro and accl" << endl;
                stamp = ( uint64_t )( ( ( double )accl.timestamp + ( double )gyro.timestamp ) / 2.0 );
            }
            else
            {
                stamp = accl.timestamp;
            }
            stream << std::to_string( stamp );

            stream << "," << gyro.data[0];
            stream << "," << gyro.data[1];
            stream << "," << gyro.data[2];

            stream << "," << accl.data[0];
            stream << "," << accl.data[1];
            stream << "," << accl.data[2] << endl;

            accl_queue.pop_front();
            gyro_queue.pop_front();
        }

        stream.close();

        LOG_INFO << "Saved IMU data to: " << imu_file << endl;
        cout << endl;
    }


    /// 解析视频数据

    if ( ignore_images )
    {
        LOG_INFO << "Image data is set to be ignored. Skipping image extraction." << endl;
        return 0;
    }

    imu_extractor.printVideoFrameRate();

    vector<uint64_t> image_stamps;
    imu_extractor.getImageStamps( image_stamps );
    LOG_INFO << "Image stamps num: " << image_stamps.size() << endl;

    VideoExtractor video_extractor( gopro_video_path );
    if ( !video_extractor.isOk() )
    {
        return 0;
    }
    {
        uint32_t gpmf_frame_count = imu_extractor.getImageCount();
        uint32_t ffmpeg_frame_count = video_extractor.getFrameCount();
        if ( gpmf_frame_count != ffmpeg_frame_count )
        {
            LOG_ERROR << "gpmf frame count: [" << gpmf_frame_count << "]"
                      << " should match ffmpeg frame count: [" << ffmpeg_frame_count << "]" << endl;
            return -1;
        }

        uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
        uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

        if ( ffmpeg_video_time != gpmf_video_time )
        {
            LOG_ERROR << "gpmf video creation time: [" << gpmf_video_time << "]"
                      << " should match ffmpeg video creation time: [" << ffmpeg_video_time << "]" << endl;
            return -1;
        }
    }

    /// 保存图片
    {
        const std::string image_folder = data_save_dir + "/cam0/data";
        if ( !std::filesystem::is_directory( image_folder ) )
        {
            std::filesystem::create_directories( image_folder );
        }
        const std::string list_file = image_folder + "/../data.csv";

        video_extractor.extractFrames( image_folder, list_file, image_stamps, scale, use_grayscale, display_images );
    }

    return 0;
}