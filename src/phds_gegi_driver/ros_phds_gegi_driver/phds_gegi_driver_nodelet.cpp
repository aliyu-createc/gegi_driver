//
// Created by createc on 10/09/2020.
//

#include <phds_gegi_driver/phds_gegi_driver.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <memory> // make_unique, unique_ptr

namespace phds_gegi_driver
{
    class PhdsGegiDriverNodelet :  public nodelet::Nodelet
    {
    protected:
        virtual void onInit()
        {
            phds_gegi_driver_ = std::make_unique<PhdsGegiDriver>(
                    getNodeHandle(), getPrivateNodeHandle());
        }
    private:
        std::unique_ptr<PhdsGegiDriver> phds_gegi_driver_;
    };
}

PLUGINLIB_EXPORT_CLASS(phds_gegi_driver::PhdsGegiDriverNodelet, nodelet::Nodelet)