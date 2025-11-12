rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr micPublisher;				 /* Mic publisher for volume picked up by mic */
rclcpp::TimerBase::SharedPtr micTimer;
	micPublisher = this->create_publisher<std_msgs::msg::Float32>("/mic", 5);	
	micTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&SensorPublisher::pubMic, this));	

    void SensorPublisher::pubMic()
{
	auto micMsg = std_msgs::msg::Float32();
	micMutex.lock();
	micMsg.data = volume;
	micMutex.unlock();
	micPublisher->publish(micMsg);
}
