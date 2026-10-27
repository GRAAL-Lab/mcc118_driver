#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <memory>
#include <thread>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <daqhats/daqhats.h>
#include <mcc118_driver/daqhats_utils.h>
#include <mcc118_driver/msg/cable_tension.hpp>
#include <mcc118_driver/msg/time.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

//extern "C" {
//    #include "daqhats_utils.h"
//}

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    std::chrono::system_clock::time_point t_now_;

    // Create a ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("mcc118_node");

    // Publish measurements on this topic
    auto tension_publisher = node->create_publisher<mcc118_driver::msg::CableTension>("/mcc118/measurements", 10);
    auto sensor_publisher = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/cable_tension", 10);

    RCLCPP_INFO(node->get_logger(),"MCC 118 publisher started");
    
    // Initializing mcc118 HAT
    int result = RESULT_SUCCESS;
    uint8_t address = 0;
    char c;
    char display_header[512];
    int i;
    char channel_string[512];
    char options_str[512];

    // Set the channel mask which is used by the library function
    // mcc118_a_in_scan_start to specify the channels to acquire.
    uint8_t channel_mask = {CHAN0 | CHAN1};
    
    // The functions below, will parse the channel mask into a
    // character string for display purposes.
    convert_chan_mask_to_string(channel_mask, channel_string);
    
    uint32_t samples_per_channel = 0;
    int max_channel_array_length = mcc118_info()->NUM_AI_CHANNELS;
    int channel_array[max_channel_array_length];
    uint8_t num_channels = convert_chan_mask_to_array(channel_mask, channel_array);

    uint32_t internal_buffer_size_samples = 0;
    uint32_t user_buffer_size = 1000 * num_channels;
    double read_buf[user_buffer_size];
    int total_samples_read = 0;

    int32_t read_request_size = READ_ALL_AVAILABLE;

    // When doing a continuous scan, the timeout value will be ignored in the
    // call to mcc118_a_in_scan_read because we will be requesting that all
    // available samples (up to the default buffer size) be returned.
    double timeout = 5.0;

    double scan_rate = 1000.0;
    double actual_scan_rate = 0.0;
    mcc118_a_in_scan_actual_rate(num_channels, scan_rate, &actual_scan_rate);

    // Set reading option here
    uint32_t options = OPTS_CONTINUOUS;// OPTS_NOCALIBRATEDATA | OPTS_NOSCALEDATA;

    uint16_t read_status = 0;
    uint32_t samples_read_per_channel = 0;
    
    // Channel 0 calibration
    double slope_n = 1.04517611026;
    double offset_n = -94.253128;
    printf("setting slop:  %f\n", slope_n);
    printf("setting calibration offset:  %f\n", offset_n);
    int re;

    // Select an MCC118 HAT device to use.
    if (select_hat_device(HAT_ID_MCC_118, &address))
        return -1; // Error getting device.
    
    printf ("\nSelected MCC 118 device at address %d\n", address);

    // Open a connection to the device.
    result = mcc118_open(address);
    //STOP_ON_ERROR(result);

    convert_options_to_string(options, options_str);
    // convert_chan_mask_to_string(channel_mask, channel_string);

    printf("\nMCC 118 tension scan node\n");
    printf("    Channels: %s\n", channel_string);
    printf("    Requested scan rate: %-10.2f\n", scan_rate);
    printf("    Actual scan rate: %-10.2f\n", actual_scan_rate);
    printf("    Options: %s\n", options_str);

    printf("\nPress ENTER to continue ...\n");
    scanf("%c", &c);
    
    re = mcc118_calibration_coefficient_write(address, CHAN1, slope_n, offset_n);
    printf("Write result: %d\n", re);
    STOP_ON_ERROR(re);
    
    if (re == RESULT_SUCCESS)
        printf("Writing coefficient is SUCCESS!");

    // Configure and start the scan.
    // Since the continuous option is being used, the samples_per_channel 
    // parameter is ignored if the value is less than the default internal
    // buffer size (10000 * num_channels in this case). If a larger internal
    // buffer size is desired, set the value of this parameter accordingly.
    
    result = mcc118_a_in_scan_start(address, channel_mask, samples_per_channel, scan_rate, options);
    STOP_ON_ERROR(result);
    STOP_ON_ERROR(mcc118_a_in_scan_buffer_size(address, &internal_buffer_size_samples));
    printf("Internal data buffer size:  %d\n", internal_buffer_size_samples);
    printf("\nStarting scan ... Press ENTER to stop\n\n");

    // Create the header containing the column names.
    strcpy(display_header, "Samples Read    Scan Count    ");
    for (i = 0; i < num_channels; i++)
    {
        sprintf(channel_string, "Channel %d   ", channel_array[i]);
        strcat(display_header, channel_string);
    }
    strcat(display_header, "\n");
    printf("%s", display_header);

    // Continuously update the display value until enter key is pressed
    do 
    {
        // Since the read_request_size is set to -1 (READ_ALL_AVAILABLE), this
        // function returns immediately with whatever samples are available (up
        // to user_buffer_size) and the timeout parameter is ignored.
        result = mcc118_a_in_scan_read(address, &read_status, read_request_size,
            timeout, read_buf, user_buffer_size, &samples_read_per_channel);
        STOP_ON_ERROR(result);
        if (read_status & STATUS_HW_OVERRUN)
        {
            printf("\n\nHardware overrun\n");
            break;
        }
        else if (read_status & STATUS_BUFFER_OVERRUN)
        {
            printf("\n\nBuffer overrun\n");
            break;
        }

        total_samples_read += samples_read_per_channel;

        // Display and publish the latest sample from each channel.
        printf("\r%12d    %10d ", samples_read_per_channel, total_samples_read);

        if (samples_read_per_channel > 0)
        {
            // Index of the most recent sample for all channels.
            int index = (samples_read_per_channel - 1) * num_channels;

            // Create a ROS 2 message.
            //std_msgs::msg::Float64MultiArray msg;
            //msg.data.resize(num_channels);
            
            mcc118_driver::msg::CableTension msg;
            geometry_msgs::msg::WrenchStamped T_sensor;
            
            t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            //long now_nanosecs =1;
            auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
    
            for (i = 0; i < num_channels; i++)
            {
                // Apply the same scaling as in your original program.
                double measurement = 20.0 * read_buf[index + i];

                // Print the measurement.
                printf(" Ch%d: %10.5f N", channel_array[i], measurement);

                // Store the measurement in the ROS 2 message.
                //msg.data[i] = measurement;
            }
            msg.stamp.sec = now_stamp_secs;
            msg.stamp.nanosec = now_stamp_nanosecs;
            msg.force_red = 20.0 * read_buf[index];
            msg.force_blue = 20.0 * read_buf[index + 1];
            
            T_sensor.header.stamp.sec = now_stamp_secs;
            T_sensor.header.stamp.nanosec = now_stamp_nanosecs;
            T_sensor.wrench.force.x = 20.0 * read_buf[index];
            T_sensor.wrench.force.y = 20.0 * read_buf[index +1];

            fflush(stdout);

            // Publish both channel measurements.
            tension_publisher->publish(msg);
            sensor_publisher->publish(T_sensor);
        }

        // Allow ROS 2 to process pending work.
        rclcpp::spin_some(node);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } 
    while ((result == RESULT_SUCCESS) && ((read_status & STATUS_RUNNING) == STATUS_RUNNING) && !enter_press());

    printf("\n");

stop:
    print_error(mcc118_a_in_scan_stop(address));
    print_error(mcc118_a_in_scan_cleanup(address));
    print_error(mcc118_close(address));
    
    rclcpp::shutdown();

    return 0;
}

