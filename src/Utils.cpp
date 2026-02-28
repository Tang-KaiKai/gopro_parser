#include "Utils.h"

#include <chrono>

#include "date.h"

namespace GoProParser
{

////////////////////////////////////////////// 函数定义 //////////////////////////////////////////////

uint64_t parseISO( const std::string &iso_date )
{
    LOG_INFO << "Parsing ISO date: [" << iso_date << "]" << std::endl;

    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp;
    std::istringstream in( iso_date );
    in >> date::parse( "%FT%TZ", tp );
    if ( in.fail() )
    {
        in.clear();
        in.exceptions( std::ios::failbit );
        in.str( iso_date );
        in >> date::parse( "%FT%T%Ez", tp );
    }

    uint64_t time = tp.time_since_epoch().count();

    return time;
}

} // namespace GoProParser