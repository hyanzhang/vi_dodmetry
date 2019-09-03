#ifndef _PARAMETER_H_
#define _PARAMETER_H_
#include "vi_odometry/common_headers.h"
namespace vi_odometry
{
class Config
{
private:
    static std::shared_ptr<Config> Ptr;
    cv::FileStorage file_;
    Config() {} //singlon mode

public:
    void setParameterFile(const std::string &file_name);
    template <typename T>
    static T get(const std::string key)
    {
        return static_cast<T>(Ptr->file_[key]);
    }
    ~Config();
};
} // namespace vi_odometry
#endif