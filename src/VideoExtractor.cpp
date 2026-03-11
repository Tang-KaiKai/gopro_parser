#include "VideoExtractor.h"
#include "Utils.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

namespace GoProParser
{

VideoExtractor::VideoExtractor( const std::string filename, bool dump_info ): video_file_( filename )
{
    pFormatContext_ = avformat_alloc_context();
    if ( !pFormatContext_ )
    {
        LOG_ERROR << "ERROR could not allocate memory for Format Context" << std::endl;
        return;
    }

    LOG_INFO << "Opening Video File: " << video_file_ << std::endl;
    if ( avformat_open_input( &pFormatContext_, video_file_.c_str(), NULL, NULL ) != 0 )
    {
        LOG_ERROR << "Could not open file: " << video_file_ << std::endl;
        return;
    }

    // Retrieve stream information
    if ( avformat_find_stream_info( pFormatContext_, NULL ) < 0 )
    {
        LOG_ERROR << "Couldn't find stream information" << std::endl;
        return;
    }

    // Dump information about file onto standard error
    if ( dump_info )
    {
        av_dump_format( pFormatContext_, 0, video_file_.c_str(), 0 );
    }

    // Find the first video stream

    AVCodecParameters *pCodecParams = NULL;
    AVDictionaryEntry *pDictEntry = NULL;
    video_stream_index_ = -1;
    std::string creation_time;

    for ( uint32_t i = 0; i < pFormatContext_->nb_streams; i++ )
    {
        pCodecParams = pFormatContext_->streams[i]->codecpar;
        if ( pCodecParams->codec_type == AVMEDIA_TYPE_VIDEO )
        {
            pDictEntry = av_dict_get( pFormatContext_->metadata, "", pDictEntry, AV_DICT_IGNORE_SUFFIX );
            while ( pDictEntry )
            {
                if ( strcmp( pDictEntry->key, "creation_time" ) == 0 )
                {
                    std::stringstream ss;
                    ss << pDictEntry->value;
                    ss >> creation_time;
                }
                pDictEntry = av_dict_get( pFormatContext_->metadata, "", pDictEntry, AV_DICT_IGNORE_SUFFIX );
            }

            video_stream_index_ = i;
            break;
        }
    }

    if ( video_stream_index_ == -1 )
    {
        LOG_ERROR << "Didn't find a video stream" << std::endl;
        return;
    }

    video_creation_time_ = parseISO( creation_time );

    auto pStream = pFormatContext_->streams[video_stream_index_];
    num_frames_ = pStream->nb_frames;

    // Get a pointer to the codec context for the video stream
    auto pCodec = avcodec_find_decoder( pStream->codecpar->codec_id );
    if ( pCodec == nullptr )
    {
        LOG_ERROR << "Unsupported codec!" << std::endl;
        return;
    }

    // Find the decoder for the video stream
    pCodecContext_ = avcodec_alloc_context3( pCodec );
    if ( !pCodecContext_ )
    {
        LOG_ERROR << "Failed to allocated memory for AVCodecContext" << std::endl;
        return;
    }

    if ( avcodec_parameters_to_context( pCodecContext_, pCodecParams ) < 0 )
    {
        LOG_ERROR << "Failed to copy codec params to codec context" << std::endl;
        return;
    }

    // Open codec
    AVDictionary *pDict = NULL;
    if ( avcodec_open2( pCodecContext_, pCodec, &pDict ) != 0 )
    {
        LOG_ERROR << "Could not open codec" << std::endl;
        return;
    }

    image_height_ = pCodecContext_->height;
    image_width_ = pCodecContext_->width;
    LOG_INFO << "video size: " << GREEN << cv::Size( image_width_, image_height_ ) << RESET
             << ", frames num: " << GREEN << num_frames_ << RESET << std::endl;

    // Close the video formatcontext
    avformat_close_input( &pFormatContext_ );

    is_ok_ = true;
}

VideoExtractor::~VideoExtractor()
{
    // Close the codec
    avcodec_close( pCodecContext_ );

    // Close the video formatcontext
    avformat_close_input( &pFormatContext_ );
}

bool VideoExtractor::extractFrames( const std::string &image_folder,
                                    const std::string &list_file,
                                    const std::vector<uint64_t> &image_stamps,
                                    float scale_factor,
                                    int interval,
                                    bool grayscale,
                                    bool display_images )
{
    if ( uint32_t( image_stamps.size() ) > num_frames_ )
    {
        LOG_ERROR << "The number of image stamps [" << image_stamps.size()
                  << "] is greater than the total number of frames [" << num_frames_ << "]." << std::endl;
        return false;
    }

    const uint32_t expected_frames = ( image_stamps.size() + interval - 1 ) / interval;
    LOG_INFO << "Expected number of frames to save: " << GREEN << expected_frames << RESET
             << ", when interval is: " << GREEN << interval << RESET << std::endl;
    if ( expected_frames == 0 )
    {
        LOG_WARNING << "No frames will be saved with the current interval setting." << std::endl;
        return false;
    }

    if ( avformat_open_input( &pFormatContext_, video_file_.c_str(), NULL, NULL ) != 0 )
    {
        LOG_ERROR << "Could not open file: " << video_file_ << std::endl;
        return false;
    }

    // Map deprecated YUVJ* formats to non-deprecated YUV* and set range explicitly
    auto src_fmt = pCodecContext_->pix_fmt;
    bool src_full_range = false;
    switch ( src_fmt )
    {
    case AV_PIX_FMT_YUVJ420P:
        src_fmt = AV_PIX_FMT_YUV420P;
        src_full_range = true;
        break;
    case AV_PIX_FMT_YUVJ422P:
        src_fmt = AV_PIX_FMT_YUV422P;
        src_full_range = true;
        break;
    case AV_PIX_FMT_YUVJ444P:
        src_fmt = AV_PIX_FMT_YUV444P;
        src_full_range = true;
        break;
    case AV_PIX_FMT_YUVJ440P:
        src_fmt = AV_PIX_FMT_YUV440P;
        src_full_range = true;
        break;
    default:
        src_full_range = ( pCodecContext_->color_range == AVCOL_RANGE_JPEG );
        break;
    }

    auto pSwsCtx = sws_getContext( pCodecContext_->width,
                                   pCodecContext_->height,
                                   src_fmt,
                                   image_width_,
                                   image_height_,
                                   AV_PIX_FMT_BGR24,
                                   SWS_BILINEAR,
                                   nullptr,
                                   nullptr,
                                   nullptr );

    if ( pSwsCtx )
    {
        const int *coeffs = sws_getCoefficients(
                                pCodecContext_->colorspace == AVCOL_SPC_UNSPECIFIED
                                ? AVCOL_SPC_BT709 : pCodecContext_->colorspace );
        // dst is RGB full range
        sws_setColorspaceDetails( pSwsCtx,
                                  coeffs, src_full_range ? 1 : 0,
                                  coeffs, 1,
                                  0, 1 << 16, 1 << 16 );
    }

    // Allocate video frame
    auto pSrcFrame = av_frame_alloc();
    if ( !pSrcFrame )
    {
        LOG_ERROR << "Could not allocate memory for AVFrame" << std::endl;
        return false;
    }

    // Allocate an AVFrame structure
    auto pDstFrame = av_frame_alloc();
    if ( !pDstFrame )
    {
        LOG_ERROR << "Cannot allocate RGB Frame" << std::endl;
        return false;
    }

    // PIX_FMT_RGB24
    // Determine required buffer size and allocate buffer
    const int bytes_num = av_image_get_buffer_size( AV_PIX_FMT_BGR24, image_width_, image_height_, 1 );
    uint8_t *buffer = ( uint8_t * )av_malloc( bytes_num * sizeof( uint8_t ) );

    // Assign appropriate parts of buffer to image planes in pDstFrame
    // Note that pDstFrame is an AVFrame, but AVFrame is a superset of AVPicture
    av_image_fill_arrays( pDstFrame->data, pDstFrame->linesize, buffer, AV_PIX_FMT_BGR24,
                          image_width_, image_height_, 1 );

    std::ofstream list_stream;
    list_stream.open( list_file, std::ofstream::out );
    list_stream << "#timestamp [ns],filename" << std::endl;
    list_stream << std::fixed << std::setprecision( 19 );

    AVPacket packet;
    uint32_t frame_count = 0;
    uint32_t save_count = 0;
    while ( av_read_frame( pFormatContext_, &packet ) >= 0 && frame_count < image_stamps.size() )
    {
        if ( packet.stream_index != video_stream_index_ )
        {
            av_packet_unref( &packet );  // Free the packet that was allocated by av_read_frame
            continue;
        }

        int ret = avcodec_send_packet( pCodecContext_, &packet );
        if ( ret >= 0 )
        {
            ret = avcodec_receive_frame( pCodecContext_, pSrcFrame );
        }

        if ( ret != 0 )
        {
            av_packet_unref( &packet );  // Free the packet that was allocated by av_read_frame
            continue;
        }

        if ( frame_count % interval == 0 )
        {
            // Convert the image from its native format to RGB
            sws_scale( pSwsCtx,
                       ( uint8_t const * const * )pSrcFrame->data,
                       pSrcFrame->linesize,
                       0,
                       pCodecContext_->height,
                       pDstFrame->data,
                       pDstFrame->linesize );

            cv::Mat raw_img( image_height_, image_width_, CV_8UC3, pDstFrame->data[0], pDstFrame->linesize[0] );
            cv::Mat final_img;
            {
                cv::Mat tmp_img;
                if ( grayscale )
                {
                    cv::cvtColor( raw_img, tmp_img, cv::COLOR_BGR2GRAY );
                }
                else
                {
                    tmp_img = raw_img;
                }

                if ( std::abs( scale_factor - 1.0 ) > 1e-2 )
                {
                    cv::resize( tmp_img, final_img, cv::Size(), scale_factor, scale_factor );
                }
                else
                {
                    final_img = tmp_img;
                }
            }

            if ( frame_count == 0 )
            {
                LOG_INFO << "video save size: " << GREEN << final_img.size() << RESET << std::endl;
            }

            if ( display_images )
            {
                cv::imshow( "GoPro Video", final_img );
                cv::waitKey( 5 );
            }

            uint64_t img_stamp = image_stamps[frame_count];
            std::string string_stamp = std::to_string( img_stamp );
            list_stream << string_stamp << "," << string_stamp + ".png" << std::endl;

            std::string path = image_folder + "/" + string_stamp + ".png";
            cv::imwrite( path, final_img );
            save_count++;
        }

        frame_count++;

        // 打印进度
        {
            const int percent = static_cast<int>( save_count * 100 / expected_frames );
            std::cout << "\rProgress: " << GREEN << percent << "% = "
                      << save_count << "/" << expected_frames << RESET << std::flush;
        }

        av_packet_unref( &packet );  // Free the packet that was allocated by av_read_frame
    }
    std::cout << std::endl;  // 换行

    list_stream.close();

    LOG_INFO << "Finished"
             << ", saved frames: " << GREEN << save_count << RESET
             << ", read frames: " << GREEN << frame_count << RESET
             << ", skip frames: " << GREEN << num_frames_ - frame_count << RESET
             << ", total frames: " << GREEN << num_frames_ << RESET << std::endl;

    // Free the RGB image
    av_free( buffer );

    // Free the frame
    av_free( pDstFrame );
    av_free( pSrcFrame );

    // Close the video file
    avformat_close_input( &pFormatContext_ );

    return true;
}

bool VideoExtractor::getFrameStamps( std::vector<uint64_t> &stamps )
{
    stamps.clear();
    stamps.reserve( num_frames_ );

    if ( avformat_open_input( &pFormatContext_, video_file_.c_str(), NULL, NULL ) != 0 )
    {
        LOG_ERROR << "Could not open file: " << video_file_ << std::endl;
        return false;
    }
    auto pStream = pFormatContext_->streams[video_stream_index_];

    // Allocate video frame
    auto pFrame = av_frame_alloc();
    if ( !pFrame )
    {
        LOG_ERROR << "Could not allocate memory for AVFrame" << std::endl;
        return false;
    }

    double global_clock;  // 单位: s
    int64_t global_video_pkt_pts = AV_NOPTS_VALUE;
    AVPacket packet;
    while ( av_read_frame( pFormatContext_, &packet ) >= 0 )
    {
        if ( packet.stream_index != video_stream_index_ )
        {
            av_packet_unref( &packet );  // Free the packet that was allocated by av_read_frame
            continue;
        }

        int ret = avcodec_send_packet( pCodecContext_, &packet );
        if ( ret >= 0 )
        {
            ret = avcodec_receive_frame( pCodecContext_, pFrame );
        }

        if ( ret != 0 )
        {
            av_packet_unref( &packet );  // Free the packet that was allocated by av_read_frame
            continue;
        }

        if ( packet.dts != AV_NOPTS_VALUE )
        {
            global_clock = pFrame->best_effort_timestamp;
            global_video_pkt_pts = packet.pts;
        }
        else if ( global_video_pkt_pts && global_video_pkt_pts != AV_NOPTS_VALUE )
        {
            global_clock = global_video_pkt_pts;
        }
        else
        {
            global_clock = 0;
        }

        double frame_delay = av_q2d( pStream->time_base );
        global_clock *= frame_delay;

        // Only if we are repeating the
        if ( pFrame->repeat_pict > 0 )
        {
            double extra_delay = pFrame->repeat_pict * ( frame_delay * 0.5 );
            global_clock += extra_delay;
        }

        uint64_t ns = ( uint64_t )( global_clock * 1000000000 );  // 转换为纳秒
        stamps.emplace_back( ns );

        av_packet_unref( &packet );  // Free the packet that was allocated by av_read_frame
    }

    // Free the frame
    av_free( pFrame );

    // Close the video file
    avformat_close_input( &pFormatContext_ );

    return true;
}


} // namespace GoProParser