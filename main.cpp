// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <signal.h>
#include <list>
#include <array>
#include <fstream>

#include <librealsense/rs.hpp>
#include "rs_sdk.h"
#include "version.h"
#include "pt_utils.hpp"
#include "pt_console_display.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace std;

// Version number of the samples
extern constexpr auto rs_sample_version = concat("VERSION: ",RS_SAMPLE_VERSION_STR);

struct Pose{
   float sequence [6][3];
};

cv::Mat Image2Mat(rs::core::image_interface *image)
    {
        cv::Mat mat;
        switch (image->query_info().format)
        {
        case rs::core::pixel_format::rgba8:
            mat = cv::Mat(image->query_info().height, image->query_info().width, CV_8UC4,
                          (void *) (image->query_data())).clone();
            cv::cvtColor(mat, mat, CV_RGBA2BGR);
            break;
        case rs::core::pixel_format::bgra8:
            mat = cv::Mat(image->query_info().height, image->query_info().width, CV_8UC4,
                          (void *) (image->query_data())).clone();
            cv::cvtColor(mat, mat, CV_BGRA2BGR);
            break;
        case rs::core::pixel_format::bgr8:
            mat = cv::Mat(image->query_info().height, image->query_info().width, CV_8UC3,
                          (void *) (image->query_data())).clone();
            break;
        case rs::core::pixel_format::rgb8:
            mat = cv::Mat(image->query_info().height, image->query_info().width, CV_8UC3,
                          (void *) (image->query_data())).clone();
            cv::cvtColor(mat, mat, CV_RGB2BGR);
            break;
        default:
            std::runtime_error("unsupported color format");
        }
        return mat;
    }

int main(int argc, char** argv)
{

    pt_utils pt_utils;
    unique_ptr<console_display::pt_console_display> console_view = move(console_display::make_console_pt_display());

    rs::core::video_module_interface::actual_module_config actualModuleConfig;
    rs::person_tracking::person_tracking_video_module_interface* ptModule = nullptr;

    // Initializing Camera and Person Tracking modules
    if(pt_utils.init_camera(actualModuleConfig) != rs::core::status_no_error)
    {
        cerr << "Error: Device is null." << endl << "Please connect a RealSense device and restart the application" << endl;
        return -1;
    }
    pt_utils.init_person_tracking(&ptModule); //Person tracking video module object

    // Enable Skeleton Detection
    ptModule->QueryConfiguration()->QuerySkeletonJoints()-> Enable();
    ptModule->QueryConfiguration()->QueryTracking()->Enable();
    ptModule->QueryConfiguration()->QueryTracking()->SetTrackingMode((Intel::RealSense::PersonTracking::PersonTrackingConfiguration::TrackingConfiguration::TrackingMode)0);

    // Configure enabled Person Tracking module
    if(ptModule->set_module_config(actualModuleConfig) != rs::core::status_no_error)
    {
        cerr<<"Error : Failed to configure the enabled Person Tracking module" << endl;
        return -1;
    }

    // Start the camera
    pt_utils.start_camera();
 
    std::list<Pose> sequences; // empty list of Pose struct


    cout << endl << "-------- Press ESC(ALE)  key to exit --------" << endl << endl;

    while(!pt_utils.user_request_exit())
    {
        rs::core::correlated_sample_set sampleSet = {};
        bool need_create_window = true;
        // Get next frame
        if (pt_utils.GetNextFrame(sampleSet) != 0)
        {
            cerr << "Error: Invalid frame" << endl;
            continue;
        }

        // Process frame
        if (ptModule->process_sample_set(sampleSet) != rs::core::status_no_error)
        {
            cerr << "Error : Failed to process sample" << endl;
            continue;
        }
    
    /*Code taken by pt_tutorial_1*/
       int numPeopleInFrame = 0;

       Intel::RealSense::PersonTracking::PersonTrackingData *trackingData = ptModule->QueryOutput();
       numPeopleInFrame = ptModule->QueryOutput()->QueryNumberOfPeople();

       if (trackingData->GetTrackingState() == Intel::RealSense::PersonTracking::PersonTrackingData::TrackingState::TRACKING_STATE_DETECTING &&
            numPeopleInFrame> 0){

           Intel::RealSense::PersonTracking::PersonTrackingData::Person* personData = trackingData->QueryPersonData(Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, 0);
           if (personData){

              cout << "Call StartTracking()" << endl;
              trackingData->StartTracking(personData->QueryTracking()->QueryId());
            }

              cout << left << setw(25) << " Number People Current Frame" << "Number of Detected Joints" << endl;
        cout << left << setw(25) << "--------------------" << "----------" << endl;

        cout << left << setw(25) << numPeopleInFrame << "To FILL" << endl << endl; 

        }
        float spheres[6][2];

        for (int index=0; index < trackingData->QueryNumberOfPeople(); index++){
            Intel::RealSense::PersonTracking::PersonTrackingData::Person *personData = nullptr;
            personData = trackingData->QueryPersonData(
                             Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, index);

            if (personData)
            {      
              Intel::RealSense::PersonTracking::PersonTrackingData::PersonTracking* personTrackingData = personData->QueryTracking();
              int id = personTrackingData->QueryId();

              Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints* personJoints = ptModule->QueryOutput()->QueryPersonDataById(id)->QuerySkeletonJoints();//id)->QuerySkeletonJoints();
        
              if (personJoints) {
                 int numDetectedJoints = personJoints->QueryNumJoints();
                 //float poses [6][3];
                 struct Pose Pos;
                 std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints()); //numDetectedJoints);
                 personJoints->QueryJoints(skeletonPoints.data());
                 cout << "Person Id " << id << " has number of joint equals to " << numDetectedJoints << endl;
                 for (int i = 0; i < numDetectedJoints; ++i)
                 {  
                        cout << "Joint " <<i << " has x world coordinate: " << skeletonPoints[i].world.x << endl;
                    if(skeletonPoints[i].confidenceWorld==100){
                        Pos.sequence[i][0] = skeletonPoints[i].world.x;
                        Pos.sequence[i][1] = skeletonPoints[i].world.y;
                        Pos.sequence[i][2] = skeletonPoints[i].world.z;

                        spheres[i][0] = skeletonPoints[i].image.x;
                        spheres[i][1] = skeletonPoints[i].image.y;
                    }
                    else{
                        Pos.sequence[i][0] = 0;
                        Pos.sequence[i][1] = 0;
                        Pos.sequence[i][2] = 0;

                        spheres[i][0] = 0;
                        spheres[i][1] = 0;
                      }
                        cout << "pose " <<i << " has element: " << Pos.sequence[i][0] <<" , "<< Pos.sequence[i][1] <<" , "<< Pos.sequence[i][2] << endl;

                        //circle(img, Point(50,50),50, Scalar(255,255,255),CV_FILLED, 8,0);
                 }
                  //cout<< poses[0] << " , "<< poses[1] << " , "<< poses[2] << " , "<< poses[3] << " , "<< poses[4] << " , "<< poses[5] << endl ;
                  
               //   Pos.sequence= poses;
                  sequences.push_back(Pos);
              }
              else{
                cout << "No person joint detected" << endl;
              }
            }
            else{
                cout << "No person data detected" << endl;
            }
        
        }
       
        // Display color image
        auto colorImage = sampleSet[rs::core::stream_type::color];
        cv::Mat renderImage = Image2Mat(colorImage);
        if(need_create_window){
            cv::namedWindow("Color image", CV_WINDOW_AUTOSIZE);
            need_create_window = false;
        }
        cv::circle(renderImage, cv::Point(spheres[0][0],spheres[0][1]),4, cv::Scalar(255,255,255),CV_FILLED, 8,0);
        cv::circle(renderImage, cv::Point(spheres[1][0],spheres[1][1]),4, cv::Scalar(255,255,0),CV_FILLED, 8,0);
        cv::circle(renderImage, cv::Point(spheres[2][0],spheres[2][1]),4, cv::Scalar(255,0,255),CV_FILLED, 8,0);
        cv::circle(renderImage, cv::Point(spheres[3][0],spheres[3][1]),4, cv::Scalar(0,255,255),CV_FILLED, 8,0);
        cv::circle(renderImage, cv::Point(spheres[4][0],spheres[4][1]),4, cv::Scalar(0,255,0),CV_FILLED, 8,0);
        cv::circle(renderImage, cv::Point(spheres[5][0],spheres[5][1]),4, cv::Scalar(0,0,255),CV_FILLED, 8,0);
          
        cv::imshow("Color image", renderImage);
        cv::waitKey(1);

        //console_view->render_color_frames(colorImage);       

        // Release color and depth image
        sampleSet.images[static_cast<uint8_t>(rs::core::stream_type::color)]->release();
        sampleSet.images[static_cast<uint8_t>(rs::core::stream_type::depth)]->release();

    }
    //Save list in a file .txt
    ofstream myfile;
    myfile.open ("example.txt" , ios::out);
    if (myfile.is_open()) { 
      if(!sequences.empty()){
        for (auto const& i : sequences) {
           //float output[6][3]= i.sequence;
           myfile << "{" ;//<< endl;
           for(int k =0; k< 6; k++){
              myfile << "[" ;//<< endl;
              for(int j=0; j<3;j++){
                myfile << i.sequence[k][j] << "\t";
              }
              myfile << "]" ;//<< endl;
           }
          myfile << "}" << endl;
        }
        cout<< "written"<<endl;
      }
    }
    myfile.close();

    pt_utils.stop_camera();
    actualModuleConfig.projection->release();
    cout << "-------- Stopping --------" << endl;
    return 0;
}

    


