#include <eureka_power_2/battery_decoder.hpp>
using namespace std::chrono_literals;
namespace battery_decoder
{
  Battery_decoder::Battery_decoder()
  :rclcpp::Node("battery_decoder"),
  voltage{0.0},
  current{0.0}
  {
    pub1=this->create_publisher<sensor_msgs::msg::BatteryState>("bat1", 10);
    pub2=this->create_publisher<sensor_msgs::msg::BatteryState>("bat1", 10);
    pub3=this->create_publisher<sensor_msgs::msg::BatteryState>("bat1", 10);
    sub= this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "can_rx", 10, std::bind(&Battery_decoder::callback, this, std::placeholders::_1));
    timer=this->create_wall_timer(
      200ms, std::bind(&Battery_decoder::publisher, this));
      RCLCPP_INFO(this->get_logger(),"battery_decoder Started!") ;
    }

    Battery_decoder::~Battery_decoder(){
    RCLCPP_INFO(this->get_logger(),"battery_decoder Killed!") ;
    }

    void Battery_decoder::callback(const std_msgs::msg::UInt8MultiArray::SharedPtr arr)
    {
        int index=static_cast<int>(arr->data[0]);
        if(0<index && 4>index)
        {
          float temp = convertFloat16ToFloat32(arr->data[1], arr->data[2]);
          if (std::isnan(temp))
          {
            temp=0;
          }
          voltage[index-1]=static_cast<float>(temp);
          temp=static_cast<float>(convertFloat16ToFloat32(arr->data[3],arr->data[4]));
          if (std::isnan(temp))
          {
            temp=0;
          }
          current[index-1]=temp;
        }
    }

    void Battery_decoder::publisher()
    {
        sensor_msgs::msg::BatteryState message;
        for (int i = 0; i < 3; i++) 
        {
        message.voltage = voltage[i];
        message.current = current[i];
        message.header.stamp = this->get_clock()->now();
        if (i==0)
        {
            pub1->publish(message);
        }
        if (i==1)
        {
            pub2->publish(message);
        }
        if (i==2)
        {
            pub3->publish(message);
        }
        }
    }

    float Battery_decoder::convertFloat16ToFloat32(uint8_t byte1, uint8_t byte2) {
        uint16_t h = (static_cast<uint16_t>(byte2) << 8) | byte1;
        uint16_t h_exp = (h & 0x7C00) >> 10; 
        uint16_t h_sig = h & 0x03FF;         
        uint32_t f;

        if (h_exp == 0) {
            if (h_sig == 0) {
                f = (h & 0x8000) << 16; 
            } else {
                h_sig <<= 1;
                while ((h_sig & 0x0400) == 0) {
                    h_sig <<= 1;
                    h_exp--;
                }
                h_exp++;
                h_sig &= ~(0x0400);
                f = ((h & 0x8000) << 16) | ((h_exp + 112) << 23) | (h_sig << 13);
            }
        } else if (h_exp == 0x1F) {
            // INF/NaN
            f = ((h & 0x8000) << 16) | 0x7F800000 | (h_sig << 13);
        } else {
            f = ((h & 0x8000) << 16) | ((h_exp + 112) << 23) | (h_sig << 13);
        }

        float result;
        std::memcpy(&result, &f, sizeof(f));
        return result;
    }



  }

  int main(int argc, char** argv)
  {
    rclcpp::init(argc, argv);


    std::shared_ptr<battery_decoder::Battery_decoder> node = std::make_shared<battery_decoder::Battery_decoder>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
  }