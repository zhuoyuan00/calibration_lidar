#include <functional>  // for function
#include <memory>      // for allocator, __shared_ptr_access
#include <string>      // for string, basic_string, operator+, to_string
#include <vector>      // for vector
#include <filesystem>

#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"       // for Menu, Horizontal, Renderer
#include "ftxui/component/component_base.hpp"  // for ComponentBase
#include "ftxui/component/component_options.hpp"  // for MenuOption
#include "ftxui/component/screen_interactive.hpp"  // for Component, ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for text, separator, bold, hcenter, vbox, hbox, gauge, Element, operator|, border

#include <lidar_camera_calib.hpp>

#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <camodocal/chessboard/Chessboard.h>
#include <camodocal/calib/CameraCalibration.h>
#include <camodocal/gpl/gpl.h>

#include <camodocal/calib/StereoCameraCalibration.h>

#include "tui_tools.h"


void lidar2lidarCalibration(const std::string& path1, const std::string& path2);

struct LidarToLidar : public BaseTUI
{
    LidarToLidar()
            :BaseTUI("Back")
            ,start{ftxui::Button("Start", [this]{calibration();})}
            ,layout{ftxui::Container::Vertical({
                                                       source_pcd_file,
                                                       target_pcd_file,
                                                       ftxui::Container::Horizontal({start, quit})
                                               })}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({ ftxui::hbox({ftxui::text(" source pcd file name : "), source_pcd_file.Render() | ftxui::flex}),
                                 ftxui::hbox({ftxui::text(" target pcd file name : "), target_pcd_file.Render() | ftxui::flex}),
                                 ftxui::separator(),
                                 ftxui::hbox({start->Render(), ftxui::filler() | ftxui::flex, quit->Render()}),
                               }) | ftxui::border;
        }));
    }

    void calibration()
    {
        std::cout<<"\n\n";
        lidar2lidarCalibration(source_pcd_file.raw_input, target_pcd_file.raw_input);
        std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n";
    }

    InputFile source_pcd_file;
    InputFile target_pcd_file;
    ftxui::Component start;
    ftxui::Component layout;
};

void nonlinear_opt_demo();

struct NonlinearOptDemoTUI : public BaseTUI
{
    NonlinearOptDemoTUI()
            :BaseTUI("Back")
            ,layout{ftxui::Container::Horizontal({quit})}
    {
        std::cout<<"\n\n";
        nonlinear_opt_demo();
        std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n";
    }

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return quit->Render() | ftxui::border | ftxui::center;
        }));
    }

    ftxui::Component layout;
};



struct CalibrationTUI : public BaseTUI
{


    CalibrationTUI()
            :BaseTUI()
            ,lidar2lidar{ftxui::Button(" Lidar To Lidar ", []{LidarToLidar{}.show();})}
            ,layout{ftxui::Container::Horizontal({lidar2lidar, quit})}
    { }

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({ftxui::text("Calibration Kit") | ftxui::center,
                ftxui::separator(),
                ftxui::hbox({lidar2lidar->Render()}),
                ftxui::separator(),
                quit->Render() | ftxui::center,
            }) | ftxui::border | ftxui::center;
        }));
    }

    std::string a;

    ftxui::Component lidar2lidar;
    ftxui::Component layout;

};

int main()
{
    CalibrationTUI{}.show();
}
