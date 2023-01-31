#include "yolo_server.h"
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/StreamSocket.h>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>

#define DRAWING

using std::cin;                      
using std::cout;                     
using std::cerr;                     
using std::endl;                     
using std::flush;                    
using std::getline;     
using std::stoi;                                
using std::string;                   
using std::vector;
using std::to_string;
using namespace cv;
using Poco::Net::ServerSocket;       
using Poco::Net::SocketAddress;      
using Poco::Net::SocketInputStream;  
using Poco::Net::SocketOutputStream; 
using Poco::Net::StreamSocket;

float table_bottomr_x;
float table_bottoml_x;
float table_topr_x;
float table_topl_x;
float max_tabley;
float min_tabley;
float rack_bottomr_x;
float rack_bottoml_x;
float rack_topr_x;
float rack_topl_x;
float max_racky;
float min_racky;
float table_half;
cv::Mat img_from_camera;
rs2_intrinsics align_intrinsics;
Yolo *yolov4;


int drawBoundingBox(cv::Mat& image, bbox_t& boundingBox, const int& location);

vector<string> getClassName(std::string fileName) {
    std::ifstream file(fileName);
     if (!file.is_open()) 
    {
        cerr << "Could not open the file - '" << fileName << "'" << endl;
    }
    std::vector<std::string> ret;
    std::string buf;
    while(std::getline(file, buf)) {
        ret.push_back(buf);
    }
    return ret;
}
vector<string> names;
// vector<string> names = getClassName("/home/gpm-server/gpm/gpm_jssp/setting/gpm.names");
// vector<string> names = getClassName("/home/vision1/gpm/gpm_jssp/setting/gpm.names");
// vector<string> names = getClassName("/home/vision2/gpm/gpm_jssp/setting/gpm.names");

int main(int argc, char* argv[])
{
    switch (std::stoi(argv[1]))
    {
        case 0:
        {
            names = getClassName("/home/gpm-server/gpm/gpm_jssp/setting/gpm.names");
            break;
        }
        case 1:
        {
            names = getClassName("/home/vision1/gpm/gpm_jssp/setting/gpm.names");
            break;
        }
        case 2:
        {
            names = getClassName("/home/vision2/gpm/gpm_jssp/setting/gpm.names");
            break;
        }
    }
    // ros::init(argc, argv, "yolo1");
    yolov4 = Yolo::GetYolo(std::stoi(argv[1]));
    cout << "Get Yolo." << endl;

    string filename;
    // string filename("/home/gpm-server/server_vision/src/vision1.txt");
    // string filename("/home/vision1/May_ws/src/vision.txt");
    // string filename("/home/vision2/May_ws/src/vision.txt");
    switch (std::stoi(argv[1]))
    {
        case 0:
        {
            filename = "/home/gpm-server/server_vision/src/vision1.txt";
            break;
        }
        case 1:
        {
            filename = "/home/vision1/May_ws/src/vision.txt";
            break;
        }
        case 2:
        {
            filename = "/home/vision2/May_ws/src/vision.txt";
            break;
        }
    }
    std::ifstream input_file(filename, std::ios::in);
    if (!input_file.is_open()) 
    {
        cerr << "Could not open the file - '" << filename << "'" << endl;
        return EXIT_FAILURE;
    }
    vector<float> tmp;
    while (!input_file.eof()) 
    {
        float number;
        input_file >> number;
        tmp.push_back(number);
    }
    input_file.close();
    table_bottomr_x = tmp[0];
    table_bottoml_x = tmp[1];
    table_topr_x = tmp[2];
    table_topl_x = tmp[3];
    max_tabley = tmp[4];
    min_tabley = tmp[5];
    rack_bottomr_x = tmp[6];
    rack_bottoml_x = tmp[7];
    rack_topr_x = tmp[8];
    rack_topl_x = tmp[9];
    max_racky = tmp[10];
    min_racky = tmp[11];
    table_half = tmp[12];
    std::vector<bbox_t> predict_result;
    Poco::Net::SocketAddress addr("0.0.0.0", 3000);
    Poco::Net::ServerSocket serverSocket(addr);

    cout << "Listen on " << addr.toString() << endl;

    Poco::Net::SocketAddress clientAddr;
    Poco::Net::StreamSocket client = serverSocket.acceptConnection(clientAddr);

    cout << "New Connection from " << clientAddr.toString() << endl;
    while (true) 
    {
    	SocketOutputStream sout(client);
    	SocketInputStream sin(client);
	    std::stringstream tmpString;
        string s;
        getline(sin, s);
        cout << s << endl;
        if (!sin) 
        {   
            client = serverSocket.acceptConnection(clientAddr);
            cout << "New Connection from " << clientAddr.toString() << endl;
        }
        yolov4->Realsense_stream_update();
        system("clear");
        cout << "Get Ready." << endl;
        cout << "Accept Connection." << endl;
        img_from_camera = yolov4->Get_RGBimage();
        // cout << "rows: " <<  img_from_camera.rows <<" cols: " << img_from_camera.cols <<endl;
        yolov4->Get_CameraIntrin(align_intrinsics);
        predict_result = yolov4->detector->detect(img_from_camera, 0.7);
        cout << predict_result.size() << endl;
        for (auto p : predict_result) 
        {
            int location_label = drawBoundingBox(img_from_camera, p, std::stoi(argv[1]));
            if (p.x_3d != 0.f)
            {
                tmpString << to_string(p.obj_id) + " " + to_string(p.x_3d) + " " + to_string(p.y_3d) + " " + to_string(p.z_3d) + " " + to_string(location_label) + " ";
                cout << names[p.obj_id] <<  " " << p.x_3d << " " << p.y_3d << " " << location_label << " " << p. << endl;
            }
        }
        if (predict_result.size() < 1)
        {
            sout << "None" << endl;
            // cout << "string: " << "None" << endl;
        }
        else 
        {
            sout << tmpString.str() << endl;
            // cout << "string: " << tmpString.str() << endl;
        }
        if (!sout) 
        {
            client = serverSocket.acceptConnection(clientAddr);
            cout << "New Connection from " << clientAddr.toString() << endl;
        }
        #ifdef DRAWING
        cv::line(img_from_camera, cv::Point(table_bottomr_x, max_tabley), cv::Point(table_bottoml_x, max_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::line(img_from_camera, cv::Point(table_bottoml_x, max_tabley), cv::Point(table_topl_x, min_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::line(img_from_camera, cv::Point(table_topl_x, min_tabley), cv::Point(table_topr_x, min_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::line(img_from_camera, cv::Point(table_topr_x, min_tabley), cv::Point(table_bottomr_x, max_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        
        // cv::line(img_from_camera, cv::Point(rack_bottomr_x, max_racky), cv::Point(rack_bottoml_x, max_racky), cv::Scalar(0, 0, 255), 5, CV_AA);
        // cv::line(img_from_camera, cv::Point(rack_bottoml_x, max_racky), cv::Point(rack_topl_x, min_racky), cv::Scalar(0, 0, 255), 5, CV_AA);
        // cv::line(img_from_camera, cv::Point(rack_topl_x, min_racky), cv::Point(rack_topr_x, min_racky), cv::Scalar(0, 0, 255), 5, CV_AA);
        // cv::line(img_from_camera, cv::Point(rack_topr_x, min_racky), cv::Point(rack_bottomr_x, max_racky), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::imshow("Color Image", img_from_camera);
        cv::waitKey(100);
        #endif
	}
    cout << "Disconnect to " << clientAddr.toString() << endl;
    return 0;
}
/** Draw a bounding box onto a Mat, include drawing it's class name. */
int drawBoundingBox(cv::Mat& image, bbox_t& boundingBox, const int& location)
{
#ifdef DRAWING
    cv::Rect rect(boundingBox.x, boundingBox.y, boundingBox.w, boundingBox.h);
    // Random select a color of bounding box
    int r = 50 + ((43 * (boundingBox.obj_id + 1)) % 150);
    int g = 50 + ((97 * (boundingBox.obj_id + 1)) % 150);
    int b = 50 + ((37 * (boundingBox.obj_id + 1)) % 150);
    cv::Scalar color(b, g, r);
    cv::rectangle(image, rect, color, 2);
#endif
    // if (rack_bottoml_x < (boundingBox.x + boundingBox.w / 2) && (boundingBox.x + boundingBox.w / 2) < rack_bottomr_x && min_racky < (boundingBox.y + boundingBox.h / 2) && (boundingBox.y + boundingBox.h / 2) < max_racky) {
    //     float a,b,a1,b1;
    //     a = (rack_bottoml_x-rack_topl_x)/(max_racky-min_racky);
    //     b = rack_bottoml_x - a * max_racky;
    //     a1 = (rack_topr_x-rack_bottomr_x)/(min_racky-max_racky);
    //     b1 = rack_topr_x - a1 * min_racky;
    //     float x_left = (boundingBox.y + boundingBox.h)*a + b;
    //     float x_right = (boundingBox.y + boundingBox.h)*a1 + b1;
    //     float x_top = rack_topl_x + (rack_topr_x - rack_topl_x)*abs((x_right - boundingBox.x - boundingBox.w / 2))/abs((x_right - x_left));
    //     float x_bottom = rack_bottoml_x + (rack_bottomr_x - rack_bottoml_x)*abs((x_right - boundingBox.x - boundingBox.w / 2))/abs((x_right - x_left));
    //     // cout << "x_pixel: " << (boundingBox.x + boundingBox.w / 2) << "ratiol: "<< (x_right - boundingBox.x - boundingBox.w / 2)/(x_right - x_left) <<endl;
    //     // cout << "x_left: " << x_left << "x_right: " << x_right << "x_top: " << x_top << "x_bottom: " << x_bottom <<endl;
    //     // boundingBox.x_3d = 1880 + 445 + (-445)*(sqrt(pow((boundingBox.x + boundingBox.w / 2 - x_top),2)+pow((boundingBox.y + boundingBox.h - min_racky),2)))/(sqrt(pow((x_top - x_bottom),2)+pow((max_racky - min_racky),2)));
    //     boundingBox.x_3d = 1880 + 445 + (-445)*(boundingBox.y + boundingBox.h - min_racky)/(max_racky - min_racky);
    //     boundingBox.y_3d = 450 + (-900)*(x_right - boundingBox.x - boundingBox.w / 2)/(x_right - x_left);
    //     boundingBox.z_3d = 0;
    //     #ifdef DRAWING
    //     // cv::putText(image, "( " + to_string(boundingBox.x_3d) + " " + to_string(boundingBox.y_3d) + " " + to_string(boundingBox.z_3d) + " )", rect.tl() + cv::Point(0, 20), 0, 0.7, color, 2);
    //     cv::putText(image, names[boundingBox.obj_id] + "Y: " + to_string(boundingBox.y_3d), rect.tl() + cv::Point(0, -5), 0, 0.3, color, 2);
    //     #endif
    //     return 3;
    // }
    if (table_bottoml_x < (boundingBox.x + boundingBox.w / 2) && (boundingBox.x + boundingBox.w / 2) < table_bottomr_x && min_tabley < (boundingBox.y + 2*boundingBox.h/3) && (boundingBox.y + 2*boundingBox.h/3) < max_tabley)
    {
        float a,b,a1,b1;
        a = (table_bottoml_x-table_topl_x)/(max_tabley-min_tabley);
        b = table_bottoml_x - a * max_tabley;
        a1 = (table_topr_x-table_bottomr_x)/(min_tabley-max_tabley);
        b1 = table_topr_x - a1 * min_tabley;
        float x_left = (boundingBox.y + boundingBox.h)*a + b;
        float x_right = (boundingBox.y + boundingBox.h)*a1 + b1;
        float x_top = table_topl_x + (table_topr_x - table_topl_x)*abs((boundingBox.x + boundingBox.w / 2 - x_left))/abs((x_right - x_left));
        float x_bottom = table_bottoml_x + (table_bottomr_x - table_bottoml_x)*abs((boundingBox.x + boundingBox.w / 2 - x_left))/abs((x_right - x_left));
        // cout << "x_pixel: " << (boundingBox.x + boundingBox.w / 2) << "ratiol: "<< (boundingBox.x + boundingBox.w / 2 - x_left)/(x_right - x_left) <<endl;
        // cout << "x_left: " << x_left << "x_right: " << x_right << "x_top: " << x_top << "x_bottom: " << x_bottom <<endl;
        // cout << "x: " << (boundingBox.x + boundingBox.w / 2) << "y: " << (boundingBox.y + boundingBox.h) << " " << (boundingBox.y + boundingBox.h-305.0)/105.0 << endl;
        // boundingBox.x_3d = 400 + (-800)*(sqrt(pow((boundingBox.x + boundingBox.w / 2 - x_top),2)+pow((boundingBox.y + boundingBox.h - min_tabley),2)))/(sqrt(pow((x_top - x_bottom),2)+pow((max_tabley - min_tabley),2)));
        boundingBox.y_3d = 750 + (-1500)*(boundingBox.x + boundingBox.w / 2 - x_left)/(x_right - x_left);
        boundingBox.z_3d = 0;
        #ifdef DRAWING
        // cv::putText(image, "( " + to_string(boundingBox.x_3d) + " " + to_string(boundingBox.y_3d) + " " + to_string(boundingBox.z_3d) + " )", rect.tl() + cv::Point(0, 20), 0, 0.7, color, 2);
        cv::putText(image, names[boundingBox.obj_id] + "Y: " + to_string(boundingBox.y_3d), rect.tl() + cv::Point(0, -5), 0, 0.3, color, 2);
        #endif
        if(boundingBox.y + boundingBox.h <= table_half){ 
            boundingBox.x_3d = 400 + (-400)*(min_tabley - boundingBox.y - boundingBox.h/2)/(min_tabley - table_half);
            switch (location)
            {
                case 1:
                    return 2;//vision1
                case 2:
                    return 1;//vision2
            }
        }
        else{
            boundingBox.x_3d = 0 + (-400)*(table_half - boundingBox.y - boundingBox.h/2)/(table_half - max_tabley);
            switch (location)
            {
                case 1:
                    return 1;//vision1
                case 2:
                    return 2;//vision2
            }
        }
    }
    else
    {
        boundingBox.z_3d = 0;
        boundingBox.x_3d = 0;
        boundingBox.y_3d = 0;
        return 0;
    }
}