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
    LOG_INFO << "Opening Video File: " << video_file_ << std::endl;
    pFormatContext_ = avformat_alloc_context();
    if ( !pFormatContext_ )
    {
        LOG_ERROR << "ERROR could not allocate memory for Format Context" << std::endl;
        return;
    }

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
    video_stream_index_ = -1;
    std::string creation_time;

    for ( uint32_t i = 0; i < pFormatContext_->nb_streams; i++ )
    {
        pCodecParams_ = pFormatContext_->streams[i]->codecpar;
        if ( pCodecParams_->codec_type == AVMEDIA_TYPE_VIDEO )
        {
            pDictEntry_ = av_dict_get( pFormatContext_->metadata, "", pDictEntry_, AV_DICT_IGNORE_SUFFIX );
            while ( pDictEntry_ )
            {
                if ( strcmp( pDictEntry_->key, "creation_time" ) == 0 )
                {
                    std::stringstream ss;
                    ss << pDictEntry_->value;
                    ss >> creation_time;
                }
                pDictEntry_ = av_dict_get( pFormatContext_->metadata, "", pDictEntry_, AV_DICT_IGNORE_SUFFIX );
            }

            video_stream_index_ = i;
            break;
        }
    }

    video_creation_time_ = parseISO( creation_time );
    if ( video_stream_index_ == -1 )
    {
        LOG_ERROR << "Didn't find a video stream" << std::endl;
        return;
    }

    pStream_ = pFormatContext_->streams[video_stream_index_];
    num_frames_ = pStream_->nb_frames;

    // Get a pointer to the codec context for the video stream
    pCodec_ = avcodec_find_decoder( pFormatContext_->streams[video_stream_index_]->codecpar->codec_id );

    // Find the decoder for the video stream
    if ( pCodec_ == nullptr )
    {
        LOG_ERROR << "Unsupported codec!" << std::endl;
        return;
    }

    pCodecContext_ = avcodec_alloc_context3( pCodec_ );
    if ( !pCodecContext_ )
    {
        LOG_ERROR << "Failed to allocated memory for AVCodecContext" << std::endl;
        return;
    }

    if ( avcodec_parameters_to_context( pCodecContext_, pCodecParams_ ) < 0 )
    {
        LOG_ERROR << "Failed to copy codec params to codec context" << std::endl;
        return;
    }

    // Open codec
    if ( avcodec_open2( pCodecContext_, pCodec_, &pDict_ ) < 0 )
    {
        LOG_ERROR << "Could not open codec" << std::endl;
        return;
    }

    // Allocate video frame
    pFrame_ = av_frame_alloc();

    // Allocate an AVFrame structure
    pFrameRGB_ = av_frame_alloc();
    if ( pFrameRGB_ == nullptr )
    {
        LOG_ERROR << "Cannot allocate RGB Frame" << std::endl;
        return;
    }

    image_height_ = pCodecContext_->height;
    image_width_ = pCodecContext_->width;
    LOG_INFO << "video raw size: " << GREEN << cv::Size( image_width_, image_height_ ) << RESET << std::endl;

    // Close the video formatcontext
    avformat_close_input( &pFormatContext_ );

    is_ok_ = true;
}

VideoExtractor::~VideoExtractor()
{
    // Free the RGB image
    av_free( pFrameRGB_ );

    // Free the YUV frame
    av_free( pFrame_ );

    // Close the codec
    avcodec_close( pCodecContext_ );

    // Close the format context
    avformat_close_input( &pFormatContext_ );
}

int VideoExtractor::extractFrames( const std::string &image_folder,
                                   const std::string &list_file,
                                   const std::vector<uint64_t> &image_stamps,
                                   float scale_factor,
                                   bool grayscale,
                                   bool display_images )
{
    std::ofstream list_stream;
    list_stream.open( list_file, std::ofstream::out );
    list_stream << "#timestamp [ns],filename" << std::endl;
    list_stream << std::fixed << std::setprecision( 19 );

    if ( avformat_open_input( &pFormatContext_, video_file_.c_str(), NULL, NULL ) != 0 )
    {
        LOG_ERROR << "Could not open file: " << video_file_ << std::endl;
        return -1;
    }
    pStream_ = pFormatContext_->streams[video_stream_index_];

    // PIX_FMT_RGB24
    // Determine required buffer size and allocate buffer
    int numBytes = av_image_get_buffer_size( AV_PIX_FMT_RGB24, image_width_, image_height_, 1 );
    uint8_t *buffer = ( uint8_t * )av_malloc( numBytes * sizeof( uint8_t ) );

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

    pSwsCtx_ = sws_getContext( pCodecContext_->width,
                               pCodecContext_->height,
                               src_fmt,
                               image_width_,
                               image_height_,
                               AV_PIX_FMT_RGB24,
                               SWS_BILINEAR,
                               nullptr,
                               nullptr,
                               nullptr );

    if ( pSwsCtx_ )
    {
        const int *coeffs = sws_getCoefficients(
                                pCodecContext_->colorspace == AVCOL_SPC_UNSPECIFIED
                                ? AVCOL_SPC_BT709 : pCodecContext_->colorspace );
        // dst is RGB full range
        sws_setColorspaceDetails( pSwsCtx_,
                                  coeffs, src_full_range ? 1 : 0,
                                  coeffs, 1,
                                  0, 1 << 16, 1 << 16 );
    }

    // Assign appropriate parts of buffer to image planes in pFrameRGB
    // Note that pFrameRGB is an AVFrame, but AVFrame is a superset
    // of AVPicture
    av_image_fill_arrays( pFrameRGB_->data, pFrameRGB_->linesize, buffer, AV_PIX_FMT_RGB24, image_width_, image_height_, 1 );

    uint32_t frame_count = 0;
    int prev_percent = -1;
    while ( av_read_frame( pFormatContext_, &packet_ ) >= 0 && frame_count < image_stamps.size() )
    {
        if ( packet_.stream_index == video_stream_index_ )
        {
            int ret = avcodec_send_packet( pCodecContext_, &packet_ );
            if ( ret >= 0 )
            {
                ret = avcodec_receive_frame( pCodecContext_, pFrame_ );
            }

            if ( ret == 0 )
            {
                // Convert the image from its native format to RGB
                sws_scale( pSwsCtx_,
                           ( uint8_t const * const * )pFrame_->data,
                           pFrame_->linesize,
                           0,
                           pCodecContext_->height,
                           pFrameRGB_->data,
                           pFrameRGB_->linesize );

                uint64_t img_stamp = image_stamps[frame_count];
                std::string string_stamp = std::to_string( img_stamp );
                list_stream << string_stamp << "," << string_stamp + ".png" << std::endl;

                cv::Mat raw_img( image_height_, image_width_, CV_8UC3, pFrameRGB_->data[0], pFrameRGB_->linesize[0] );

                cv::Mat final_img;
                {
                    cv::Mat tmp_img;
                    if ( grayscale )
                    {
                        cv::cvtColor( raw_img, tmp_img, cv::COLOR_RGB2GRAY );
                    }
                    else
                    {
                        cv::cvtColor( raw_img, tmp_img, cv::COLOR_RGB2BGR );
                    }

                    if ( std::abs( scale_factor - 1.0 ) > 1e-3 )
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

                std::string path = image_folder + "/" + string_stamp + ".png";
                cv::imwrite( path, final_img );

                frame_count++;
            }

            const int percent = ( double )frame_count * 100.0 / ( double )image_stamps.size();
            if ( percent % 10 == 0 && prev_percent != percent )
            {
                prev_percent = percent;
                LOG_INFO << "Progress: [" << percent << "% = " << frame_count << "/" << image_stamps.size() << "]" << std::endl;
            }
        }

        // Free the packet that was allocated by av_read_frame
        av_packet_unref( &packet_ );
    }

    list_stream.close();

    LOG_INFO << GREEN << "Finished, extracted frames: " << frame_count
             << ", skip frames: " << num_frames_ - frame_count
             << ", total frames: " << num_frames_ << RESET << std::endl;

    // Free the RGB image
    av_free( buffer );

    // Close the video file
    avformat_close_input( &pFormatContext_ );

    return 0;
}

// int VideoExtractor::getFrameStamps( std::vector<uint64_t> &stamps )
// {
//     stamps.clear();

//     if ( avformat_open_input( &pFormatContext_, video_file_.c_str(), NULL, NULL ) != 0 )
//     {
//         LOG_ERROR << "Could not open file: " << video_file_ << std::endl;
//         return -1;
//     }
//     pStream_ = pFormatContext_->streams[video_stream_index_];

//     double global_clock;
//     uint64_t global_video_pkt_pts = AV_NOPTS_VALUE;
//     int decode_flag;
//     uint32_t frame_count = 0;
//     while ( av_read_frame( pFormatContext_, &packet_ ) >= 0 )
//     {
//         // Is this a packet from the video stream?
//         if ( packet_.stream_index == video_stream_index_ )
//         {
//             // Decode video frame
//             //            avcodec_send_packet(pCodecContext, &packet);
//             //            decode_flag = avcodec_receive_frame(pCodecContext, pFrameRGB);
//             avcodec_decode_video2( pCodecContext_, pFrame_, &decode_flag, &packet_ );

//             // Did we get a video frame?

//             if ( packet_.dts != AV_NOPTS_VALUE )
//             {
//                 global_clock = av_frame_get_best_effort_timestamp( pFrame_ );
//                 global_video_pkt_pts = packet_.pts;
//             }
//             else if ( global_video_pkt_pts && global_video_pkt_pts != AV_NOPTS_VALUE )
//             {
//                 global_clock = global_video_pkt_pts;
//             }
//             else
//             {
//                 global_clock = 0;
//             }

//             double frame_delay = av_q2d( pStream_->time_base );
//             global_clock *= frame_delay;

//             // Only if we are repeating the
//             if ( pFrame_->repeat_pict > 0 )
//             {
//                 double extra_delay = pFrame_->repeat_pict * ( frame_delay * 0.5 );
//                 global_clock += extra_delay;
//             }

//             uint64_t usecs = ( uint64_t )( global_clock * 1000000 );
//             // std::cout << "Stamp: " << usecs << std::endl;
//             stamps.push_back( usecs );

//             frame_count++;
//             double percent = ( double )frame_count / ( double )num_frames_;
//             // progress.write( percent );
//         }
//     }

//     // Close the video file
//     avformat_close_input( &pFormatContext_ );

//     return 0;
// }


} // namespace GoProParser