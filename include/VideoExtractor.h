#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <string>
#include <vector>


namespace GoProParser
{

class VideoExtractor
{
private:
    const std::string video_file_;
    bool is_ok_ = false;

    AVFormatContext *pFormatContext_ = NULL;
    AVCodecContext *pCodecContext_ = NULL;

    int video_stream_index_;
    uint64_t video_creation_time_;
    uint32_t image_width_;
    uint32_t image_height_;
    uint32_t num_frames_;

public:
    VideoExtractor( const std::string file, bool dump_info = false );
    ~VideoExtractor();

    bool isOk() const
    {
        return is_ok_;
    }

    uint32_t getFrameCount() const
    {
        return num_frames_;
    }

    uint64_t getVideoCreationTime() const
    {
        return video_creation_time_;
    }

    bool extractFrames( const std::string &image_folder,
                        const std::string &list_file,
                        const std::vector<uint64_t> &image_stamps,
                        float scale_factor = 1.0,
                        int interval = 1,
                        bool grayscale = false,
                        bool display_images = false );

    bool getFrameStamps( std::vector<uint64_t> &stamps );
};

} // namespace GoProParser