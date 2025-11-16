#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

namespace warrior::hardware {

class WarriorHardware : public hardware_interface::SystemInterface {
public: 
    WarriorHardware();
    ~WarriorHardware();

    void initialize();
    void shutdown();
    
    void readSensors();
    void writeActuators();

private:
    // Add private members and methods as needed
};

}  // namespace warrior::hardware