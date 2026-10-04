#include "ZedDriver.h"

using namespace rclcpp;

namespace sensors
{


    ZedDriver::ZedDriver(const NodeOptions & options,std::string json_config) : Node("zed_node",options)
    {
        // Defines the Coordinate system and unit used in this sample
        constexpr sl::COORDINATE_SYSTEM COORDINATE_SYSTEM = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
        constexpr sl::UNIT UNIT = sl::UNIT::METER;
        auto configurations = sl::readFusionConfigurationFile(json_config, COORDINATE_SYSTEM, UNIT);

        if (configurations.empty()) {
            std::cout << "Empty configuration File." << std::endl;
            return;
        }

        // Check if the ZED camera should run within the same process or if they are running on the edge.
        

        int id = 0;
        for (auto conf: configurations) 
        {
            ClientPublisher * it = new ClientPublisher(*this,id,mtx);
            id++;

            // if the ZED camera should run locally, then start a thread to handle it
            if(conf.communication_parameters.getType() == sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS){
                std::cout << "Try to open ZED " <<conf.serial_number << ".." << std::flush;
                auto state = it->open(conf);

                if (!state) {
                    std::cerr << "Could not open ZED: " << conf.input_type.getConfiguration() << ". Skipping..." << std::endl;
                    continue;
                }
                clients.push_back(it);

                std::cout << ". ready !" << std::endl;
            }
        }
        for (auto i : clients)
        {
            i->start();
        }
    };
    

}
