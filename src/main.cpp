#include "yolo_server.h"
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/StreamSocket.h>
#include <fstream>
#include <string>
#include <sstream>
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

int max_tablex;
int min_tablex;
int max_tabley;
int min_tabley;
int max_rackx;
int min_rackx;
int max_racky;
int min_racky;
int rack_depth;
cv::Mat img_from_camera;
rs2_intrinsics align_intrinsics;
Yolo *yolov4;


int drawBoundingBox(cv::Mat& image, bbox_t& boundingBox);

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

int main(int argc, char* argv[])
{
    // ros::init(argc, argv, "yolo1");
    yolov4 = Yolo::GetYolo();
    cout << "Get Yolo." << endl;

    // vector<string> names = getClassName("/home/gpm-server/api/darknet/data/coco.names");
    vector<string> names = getClassName("/home/vision1/api/darknet/data/coco.names");
    // vector<string> names = getClassName("/home/vision2/api/darknet/data/coco.names");
    // string filename("/home/gpm-server/server_vision/src/vision1.txt");
    string filename("/home/vision1/May_ws/src/vision.txt");
    // string filename("/home/vision2/May_ws/src/vision.txt");
    std::ifstream input_file(filename, std::ios::in);
    if (!input_file.is_open()) 
    {
        cerr << "Could not open the file - '" << filename << "'" << endl;
        return EXIT_FAILURE;
    }
    vector<int> tmp;
    while (!input_file.eof()) 
    {
        int number;
        input_file >> number;
        tmp.push_back(number);
    }
    input_file.close();
    max_tablex = tmp[0];
    min_tablex = tmp[1];
    max_tabley = tmp[2];
    min_tabley = tmp[3];
    max_rackx = tmp[4];
    min_rackx = tmp[5];
    max_racky = tmp[6];
    min_racky = tmp[7];
    rack_depth = tmp[8];
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
        cout << "Get Ready." << endl;
        cout << "Accept Connection." << endl;
        img_from_camera = yolov4->Get_RGBimage();
        cout << "rows: " <<  img_from_camera.rows <<" cols: " << img_from_camera.cols <<endl;
        yolov4->Get_CameraIntrin(align_intrinsics);
        predict_result = yolov4->detector->detect(img_from_camera, 0.2);
        cout << predict_result.size() << endl;
        for (auto p : predict_result) 
        {
            int location_label = drawBoundingBox(img_from_camera, p);

            tmpString << to_string(p.obj_id) + " " + to_string(p.x_3d) + " " + to_string(p.y_3d) + " " + to_string(p.z_3d) + " " + to_string(location_label) + " ";
            cout << names[p.obj_id] << " " << p.x_3d << " " << p.y_3d << " " << p.z_3d << endl;
            
        }
        if (predict_result.size() < 1)
        {
            sout << "None" << endl;
        }
        else sout << tmpString.str() << endl;
        if (!sout) 
        {
            client = serverSocket.acceptConnection(clientAddr);
            cout << "New Connection from " << clientAddr.toString() << endl;
        }
        #ifdef DRAWING
        cv::line(img_from_camera, cv::Point(min_tablex, max_tabley), cv::Point(max_tablex, max_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::line(img_from_camera, cv::Point(max_tablex, max_tabley), cv::Point(max_tablex, min_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::line(img_from_camera, cv::Point(max_tablex, min_tabley), cv::Point(min_tablex, min_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::line(img_from_camera, cv::Point(min_tablex, min_tabley), cv::Point(min_tablex, max_tabley), cv::Scalar(0, 0, 255), 5, CV_AA);
        cv::imshow("Color Image", img_from_camera);
        cv::waitKey(100);
        #endif
	}
    cout << "Disconnect to " << clientAddr.toString() << endl;
    return 0;
}
/** Draw a bounding box onto a Mat, include drawing it's class name. */
int drawBoundingBox(cv::Mat& image, bbox_t& boundingBox)
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
    if (min_rackx < (boundingBox.x + boundingBox.w / 2) && (boundingBox.x + boundingBox.w / 2) < max_rackx && min_racky < (boundingBox.y + boundingBox.h / 2) && (boundingBox.y + boundingBox.h / 2) < max_racky) {
        float camera[3];
        float pixel[2] = {boundingBox.x+boundingBox.w/2, boundingBox.y+boundingBox.h/2};
        rs2_deproject_pixel_to_point(camera, &align_intrinsics, pixel, rack_depth);
        boundingBox.z_3d = rack_depth - 4.1;
        boundingBox.x_3d = camera[0]*1000-32.5;
        boundingBox.y_3d = camera[1]*1000;
        return 2;
    }
    float depth = 0;
    yolov4->Get_Depth(boundingBox.x+boundingBox.w/2,boundingBox.y+boundingBox.h/2,depth);
    if (depth > 0) {
        float camera[3];
        float pixel[2] = {boundingBox.x+boundingBox.w/2, boundingBox.y+boundingBox.h/2};
        rs2_deproject_pixel_to_point(camera, &align_intrinsics, pixel, depth);
        boundingBox.x_3d = camera[0]*1000-32.5;
        boundingBox.y_3d = camera[1]*1000;
        boundingBox.z_3d = depth*1000-4.1;
    } 
    else {
        boundingBox.z_3d = 0;
        boundingBox.x_3d = 0;
        boundingBox.y_3d = 0;
    }
#ifdef DRAWING
    cv::putText(image, "( " + to_string(boundingBox.x_3d) + " " + to_string(boundingBox.y_3d) + " " + to_string(boundingBox.z_3d) + " )", rect.tl() + cv::Point(0, 20), 0, 0.7, color, 2);
#endif
    if (min_tablex < (boundingBox.x + boundingBox.w / 2) && (boundingBox.x + boundingBox.w / 2) < max_tablex && min_tabley < (boundingBox.y + boundingBox.h / 2) && (boundingBox.y + boundingBox.h / 2) < max_tabley)
        return 1;
    else
        return 0;
}
