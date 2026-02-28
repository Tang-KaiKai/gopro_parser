#pragma once

#include <ostream>
#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>

namespace GoProParser
{

////////////////////////////////////////////// 宏定义 //////////////////////////////////////////////

#define RESET "\033[0m"               /* Reset */
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */


////////////////////////////////////////////// 函数声明 //////////////////////////////////////////////

// 解析ISO格式的时间字符串, 返回从 1970年1月1日 到该时间的纳秒数
uint64_t parseISO( const std::string &iso_date );


////////////////////////////////////////////// 类声明 //////////////////////////////////////////////

class Tool
{
public:
    /**
     * @brief 获取当前时间的字符串
     *
     * @return std::string 当前时间字符串,格式 [M-D H:M:S.ms]
     */
    static std::string TimeStr()
    {
        // 获取当前时间点

        const auto now = std::chrono::system_clock::now();

        // 转换为 time_t 以 获取 tm 结构

        const std::time_t now_time_t = std::chrono::system_clock::to_time_t( now );
        const std::tm *now_tm = std::localtime( &now_time_t );

        // 获取毫秒部分

        const auto duration_since_epoch = now.time_since_epoch();
        const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>( duration_since_epoch ).count() % 1000;

        std::stringstream ss;
        ss << "["
           << std::setw( 2 ) << std::setfill( '0' ) << ( now_tm->tm_mon + 1 ) << "-" // 月份（加1因为tm_mon是从0开始的）
           << std::setw( 2 ) << std::setfill( '0' ) << now_tm->tm_mday << " "        // 日期（两位数字）
           << std::setw( 2 ) << std::setfill( '0' ) << now_tm->tm_hour << ":"        // 小时（两位数字）
           << std::setw( 2 ) << std::setfill( '0' ) << now_tm->tm_min << ":"         // 分钟（两位数字）
           << std::setw( 2 ) << std::setfill( '0' ) << now_tm->tm_sec << "."         // 秒（两位数字）
           << std::setw( 3 ) << std::setfill( '0' ) << milliseconds                  // 毫秒（三位数字）
           << "]";
        return ss.str();
    }

    /**
     * @brief 从命令行参数中解析参数
     *
     * @param argc 命令行参数个数
     * @param argv 命令行参数
     * @param key 参数名称
     * @param pValue 解析到的参数值
     * @return true 成功
     * @return false 失败
     */
    static bool ParseCmdLine( int argc, char **argv, const std::string &key, std::string *pValue = nullptr )
    {
        bool bFound = false;
        for ( int i = 1; i < argc; i++ )
        {
            if ( std::string( argv[i] ) == key )
            {
                bFound = true;
            }
        }

        if ( pValue )
        {
            bFound = false;
            for ( int i = 1; i < argc - 1; i++ )
            {
                if ( std::string( argv[i] ) == key )
                {
                    *pValue = argv[i + 1];
                    bFound = true;
                    break;
                }
            }
        }

        return bFound;
    }

    // 将时间戳( ns )转换为字符串(格式: HH:MM:SS.mmm)
    static std::string TimestampToStr( uint64_t timestamp )
    {
        time_t sec = static_cast<time_t>( timestamp / 1000000000 );
        uint64_t nsec = timestamp % 1000000000;
        uint64_t msec = nsec / 1000000;

        struct tm *p;
        p = localtime( &sec );

        char time_str[32];
        sprintf( time_str, "%02d:%02d:%02d.%03ld", p->tm_hour, p->tm_min, p->tm_sec, msec );

        return std::string( time_str );
    }
};

// [文件名:行号]
#define CODE_DETAILS ( std::string( "[" ) + \
                     std::string( __FILE__ ).substr( std::string( __FILE__ ).find_last_of( '/' ) + 1 ) + \
                     ":" + std::to_string( __LINE__ ) + "] " )

// 日志打印前缀( [时间][INFO][文件名:行号] )
#define LOG_INFO std::cout << Tool::TimeStr() << GREEN << "[INFO]" << RESET << CODE_DETAILS

// 日志打印前缀( [时间][WARNING][文件名:行号] )
#define LOG_WARNING std::cerr << Tool::TimeStr() << YELLOW << "[WARNING]" << RESET << CODE_DETAILS

// 日志打印前缀( [时间][ERROR][文件名:行号] )
#define LOG_ERROR std::cerr << Tool::TimeStr() << RED << "[ERROR]" << RESET << CODE_DETAILS

} // namespace GoProParser