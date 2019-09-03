#include "vi_odometry/config.h"

namespace vi_odometry
{
void Config::setParameterFile(const std::string &file_name)
{
    if (Ptr == nullptr)
    {
        Ptr = std::shared_ptr<Config>(new Config());
    }
    Ptr->file_ = cv::FileStorage(file_name.c_str(), cv::FileStorage::READ);
    if (!Ptr->file_.isOpened())
    {
        std::cout << "parameter file: " << file_name << " does not exist." << std::endl;
        Ptr->file_.release();
        return;
    }
}
Config::~Config()
{
    if (file_.isOpened())
    {
        file_.release();
    }
}
std::shared_ptr<Config> Config::Ptr = nullptr;
} // namespace vi_odometry
