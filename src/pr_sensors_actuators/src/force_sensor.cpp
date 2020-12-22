#include "pr_sensors_actuators/force_sensor.hpp"

using namespace std::chrono_literals;

namespace pr_sensors_actuators
{
    // FORCE SENSOR COMPONENT
    ForceSensor::ForceSensor(const rclcpp::NodeOptions & options)
    : Node("force_sensor", options)
    {
        //Configuración del socket
        RCLCPP_INFO(this->get_logger(), "Configurando sensor");
        socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketHandle == -1) {
		    RCLCPP_ERROR(this->get_logger(), "Error 0");
		    exit(1);
	      }

        *(uint16_t*)&request[0] = htons(0x1234); /* standard header. */
	    *(uint16_t*)&request[2] = htons(0x0042); /* per table 9.1 in Net F/T user manual. Ese es el valor de reset BIAS */
	    *(uint32_t*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
	    /*for (int i=0; i<8; i++){
		    std::cout << static_cast<int>(request[i]) << " " << std::endl;
	    }*/

	    /* Sending the request. */
        he = gethostbyname("192.168.1.1");
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT);

        err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) ); //Faltaba esta línea, por lo que no estaba conectado el socket
        if (err == -1) {
          RCLCPP_ERROR(this->get_logger(), "Error 1");
          exit(2);
        }

        send( socketHandle, request, 8, 0 );

        usleep(1000000);

        *(uint16_t*)&request[0] = htons(0x1234); /* standard header. */
        *(uint16_t*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
        *(uint32_t*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

        /*for (int i=0; i<8; i++){
          std::cout << static_cast<int>(request[i]) << " " << std::endl;
        }*/

        /* Sending the request. */
        he = gethostbyname("192.168.1.1");
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT);

        err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
        if (err == -1) {
          RCLCPP_ERROR(this->get_logger(), "Error 2");
          exit(2);
        }

        timer_ = this->create_wall_timer(10ms, std::bind(&ForceSensor::timer_callback, this));
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRForceState>("force_state", 1);
        RCLCPP_INFO(this->get_logger(), "Sensor configurado");

    }

    void ForceSensor::timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Sending request");
        send(socketHandle, request, 8, 0 );
        recv(socketHandle, response, 36, 0 );
        resp.rdt_sequence = ntohl(*(uint32_t*)&response[0]);
        resp.ft_sequence = ntohl(*(uint32_t*)&response[4]);
        resp.status = ntohl(*(uint32_t*)&response[8]);
        for( i_fuerza = 0; i_fuerza < 6; i_fuerza++ ) {
            resp.FTData[i_fuerza] = ntohl(*(int32_t*)&response[12 + i_fuerza * 4]);
        }

        auto force_msg = pr_msgs::msg::PRForceState();

        force_msg.force[0] = 1.0*resp.FTData[0]/1000000.0;
        force_msg.force[1] = 1.0*resp.FTData[1]/1000000.0;
        force_msg.force[2] = 1.0*resp.FTData[2]/1000000.0;
        force_msg.momentum[0] = 1.0*resp.FTData[3]/1000000.0;
        force_msg.momentum[1] = 1.0*resp.FTData[4]/1000000.0;
        force_msg.momentum[2] = 1.0*resp.FTData[5]/1000000.0;

        force_msg.header.stamp = this->get_clock()->now();

        publisher_->publish(force_msg);

        RCLCPP_INFO(this->get_logger(), "Sensor: %f %f %f %f %f %f",force_msg.force[0], 
                                                                    force_msg.force[1], 
                                                                    force_msg.force[2], 
                                                                    force_msg.momentum[0],
                                                                    force_msg.momentum[1],
                                                                    force_msg.momentum[2]);
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::ForceSensor)