/* C Library Headers */
#include <stdio.h>
#include <string.h>

/* Linux headers */
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()




void main(int argc, char * argv[])
{


    
    std::string ns = ros::getNamespace(); /* This is the node namespace, this is the robot's hostname*/
    
    
    rclcpp::init(arcg,argv);
    rclcpp::spin(std::make_shared<SensorPublisher>();
    rclcpp::shutdown();
    return 0;

}
