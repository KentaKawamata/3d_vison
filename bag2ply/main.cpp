#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <opencv/highgui.h>
#include <string>
int main(int argc, char* argv[] ) {
    int number = 0;
    char key;

    std::string file_name = argv[1];
    std::string output_names = "Outputs";

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_device_from_file(file_name.c_str());
    pipe.start(cfg);

    //rs2::device device = pipe.get_active_profile().get_device();
    //rs2::playback playback = device.as<rs2::playback>();
    rs2::frameset frames;
    rs2::points points;
    rs2::pointcloud pc;
    rs2::colorizer color_map;

    cv::namedWindow("Depth", CV_WINDOW_NORMAL);
    cv::namedWindow("Color", CV_WINDOW_NORMAL);

    cv::resizeWindow("Depth", 640, 480);
    cv::resizeWindow("Color", 640, 480);

    while(true) {

        if(pipe.poll_for_frames(&frames)) {

            auto frame = pipe.wait_for_frames();
            auto depth_frame = frame.get_depth_frame();
            auto color_frame = frame.get_color_frame();


            cv::Mat color_mat = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, const_cast<void *>( color_frame.get_data()));
            cv::Mat depth_mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16SC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);

            points = pc.calculate(depth_frame);
            pc.map_to(color_frame);

            depth_mat.convertTo(depth_mat, CV_8U, -255.0/10000.0, 255.0);
            cv::equalizeHist(depth_mat, depth_mat);
            //cv::applyColorMap(ir, ir, cv::COLORMAP_JET);

            cv::imshow("Depth", depth_mat);
            cv::imshow("Color", color_mat);
            const int key = cv::waitKey(30);

            if(key=='s') {
                output_names = std::to_string(number)+".ply";
                points.export_to_ply(output_names,color_frame);
                number++;
                std::cout << "save ply_file" << std::endl;
            }
            else if(key==27) {
                cv::destroyAllWindows();
                break;
            }
        }
    }
    pipe.stop();

    return 0;
}
