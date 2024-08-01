#ifndef RECORDER_HPP_
#define RECORDER_HPP_

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

class RealSenseVideoRecorder
{
public:
    RealSenseVideoRecorder()
        : recording_(false), running_(true)
    {
        // Define the directory and create it if necessary
        std::string home_directory = std::getenv("HOME");  // Get home directory
        std::filesystem::path video_directory = std::filesystem::path(home_directory) / "Gimbal_Log" / "Video";
        
        // Create the directory if it doesn't exist
        if (!std::filesystem::exists(video_directory)) {
            try {
                std::filesystem::create_directories(video_directory);
            } catch (const std::filesystem::filesystem_error& e) {
                std::cerr << "Failed to create directory: " << e.what() << std::endl;
                throw std::runtime_error("Failed to create directory");
            }
        }

        // Generate timestamp for filename
        std::time_t now = std::time(nullptr);
        std::tm tm = *std::localtime(&now);
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        std::string timestamp = ss.str();

        // Construct filename with timestamp
        output_file_ = (video_directory / (timestamp + ".mp4")).string();  // Example filename format: Video/2024-06-28_15-30-00.mp4

        // Initialize OpenCV video writer with MP4 codec
        video_writer_ = cv::VideoWriter(output_file_, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(1920, 1080));
        if (!video_writer_.isOpened()) {
            throw std::runtime_error("Failed to open video writer");
        }

        // Initialize RealSense pipeline
        rs2::config cfg;
        cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1920, 1080, rs2_format::RS2_FORMAT_BGR8, 30);
        pipe_.start(cfg);

        // Calculate the time duration for each frame
        frame_duration_ = std::chrono::milliseconds(1000 / 30);

        // Start a separate thread for frame capturing
        capture_thread_ = std::thread([this]() {
            while (running_) {
                if (recording_) {
                    auto start_time = std::chrono::steady_clock::now();

                    rs2::frameset frames;
                    try {
                        frames = pipe_.wait_for_frames();
                    } catch (const rs2::error& e) {
                        std::cerr << "Failed to get frames: " << e.what() << std::endl;
                        continue;  // Skip this frame and try again
                    }

                    auto frame = frames.get_color_frame();
                    const int w = frame.as<rs2::video_frame>().get_width();
                    const int h = frame.as<rs2::video_frame>().get_height();
                    cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

                    // Write frame to video file
                    video_writer_.write(image);

                    // Calculate elapsed time and sleep duration to maintain frame rate
                    auto end_time = std::chrono::steady_clock::now();
                    auto elapsed_time = end_time - start_time;
                    auto sleep_duration = frame_duration_ - elapsed_time;
                    if (sleep_duration > std::chrono::milliseconds(0)) {
                        std::this_thread::sleep_for(sleep_duration);
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep if not recording
                }
            }
        });
    }

    ~RealSenseVideoRecorder()
    {
        recording_ = false;
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        video_writer_.release();
        std::cout << "Recording stopped and resources released." << std::endl;
    }

    void startRecording()
    {
        recording_ = true;
        std::cout << "Started recording." << std::endl;
    }

    void stopRecording()
    {
        recording_ = false;
        std::cout << "Stopped recording." << std::endl;
    }
    
private:
    rs2::pipeline pipe_;
    cv::VideoWriter video_writer_;
    std::thread capture_thread_;
    std::atomic<bool> recording_;
    std::atomic<bool> running_;
    std::string output_file_;
    std::chrono::milliseconds frame_duration_;
};

#endif





// #include <librealsense2/rs.hpp>
// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <thread>
// #include <atomic>
// #include <ctime>
// #include <iomanip>
// #include <sstream>

// class RealSenseVideoRecorder
// {
// public:
//     RealSenseVideoRecorder()
//         : frame_count_(0), recording_(false), running_(true)
//     {
//         // Generate timestamp for filename
//         std::time_t now = std::time(nullptr);
//         std::tm tm = *std::localtime(&now);
//         std::stringstream ss;
//         ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
//         std::string timestamp = ss.str();

//         // Construct filename with timestamp
//         output_file_ = timestamp + ".mp4";  // Example filename format: output_2024-06-28_15-30-00.mp4

//         // Initialize OpenCV video writer with MP4 codec
//         video_writer_ = cv::VideoWriter(output_file_, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(1920, 1080));

//         // Initialize RealSense pipeline
//         rs2::config cfg;
//         cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1920, 1080, rs2_format::RS2_FORMAT_BGR8, 30);
//         pipe_.start(cfg);

//         // Start a separate thread for frame capturing
//         capture_thread_ = std::thread([this]() {
//             while (running_) {
//                 if (recording_) {
//                     rs2::frameset frames = pipe_.wait_for_frames();
//                     auto frame = frames.get_color_frame();
//                     const int w = frame.as<rs2::video_frame>().get_width();
//                     const int h = frame.as<rs2::video_frame>().get_height();
//                     cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

//                     // Write frame to video file
//                     video_writer_.write(image);
//                     frame_count_++;
//                     std::cout << "Saved frame " << frame_count_ << std::endl;
//                 }
//                 // Optional: Sleep to reduce CPU usage (can be adjusted or removed as needed)
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             }
//         });
//     }

//     ~RealSenseVideoRecorder()
//     {
//         stopRecording();  // Ensure recording is stopped
//         running_ = false;
//         if (capture_thread_.joinable()) {
//             capture_thread_.join();
//         }
//         video_writer_.release();
//         std::cout << "Recording stopped and resources released." << std::endl;
//     }

//     void startRecording()
//     {
//         recording_ = true;
//         std::cout << "Started recording." << std::endl;
//     }

//     void stopRecording()
//     {
//         recording_ = false;
//         std::cout << "Stopped recording." << std::endl;
//     }

// private:
//     rs2::pipeline pipe_;
//     cv::VideoWriter video_writer_;
//     std::thread capture_thread_;
//     int frame_count_;
//     std::atomic<bool> recording_;
//     std::atomic<bool> running_;
//     std::string output_file_;
// };