#include "charuco_pose.h"
#include <iostream>
#include <stdexcept>
static void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
int main(){
    using namespace teleop_charuco;
    Spec spec{10,16,.03,.022,1};
    require(valid(spec),"valid board rejected");
    require(!valid(Spec{10,16,.03,.022,0}),"dictionary capacity not checked");
    cv::Mat printed;
#ifdef TELEOP_MODERN_CHARUCO
    auto dictionary=cv::aruco::getPredefinedDictionary(spec.dictionary);
    cv::aruco::CharucoBoard board(cv::Size(spec.squares_x,spec.squares_y),spec.square_m,spec.marker_m,dictionary);
    board.setLegacyPattern(true);board.generateImage(cv::Size(1000,1600),printed,0,1);
#else
    auto dictionary=cv::aruco::getPredefinedDictionary(spec.dictionary);
    auto board=cv::aruco::CharucoBoard::create(spec.squares_x,spec.squares_y,spec.square_m,spec.marker_m,dictionary);
    board->draw(cv::Size(1000,1600),printed,0,1);
#endif
    cv::Mat K=(cv::Mat_<double>(3,3)<<900,0,640,0,902,360,0,0,1);
    cv::Vec3d rv(.2,-.3,.1),tv(-.15,-.24,1.05);
    std::vector<cv::Point3f> outline{{0,0,0},{.3f,0,0},{.3f,.48f,0},{0,.48f,0}};
    std::vector<cv::Point2f> destination;cv::projectPoints(outline,rv,tv,K,cv::noArray(),destination);
    std::vector<cv::Point2f> source{{0,0},{999,0},{999,1599},{0,1599}};
    cv::Mat image;cv::warpPerspective(printed,image,cv::getPerspectiveTransform(source,destination),cv::Size(1280,720),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255));
    for(bool partial:{false,true}){
        cv::Mat frame=image.clone();if(partial)cv::rectangle(frame,cv::Rect(0,460,1280,260),cv::Scalar(255),-1);
        cv::Matx44d pose;double error;int corners;
        require(detect_pose(frame,K,spec,pose,error,corners),"board pose not detected");
        cv::Vec3d measured(pose(0,3),pose(1,3),pose(2,3));
        require(cv::norm(measured-tv)<.005,"metric board translation inaccurate");
        cv::Mat R;cv::Rodrigues(rv,R);cv::Matx33d estimated;
        for(int y=0;y<3;y++)for(int x=0;x<3;x++)estimated(y,x)=pose(y,x);
        cv::Mat delta=cv::Mat(estimated)*R.t();double angle=std::acos(std::max(-1.,std::min(1.,(cv::trace(delta)[0]-1)/2)))*180/CV_PI;
        require(angle<2,"board rotation inaccurate");
        std::cout<<"partial="<<partial<<" corners="<<corners<<" rms="<<error<<" translation_mm="<<cv::norm(measured-tv)*1000<<" rotation_deg="<<angle<<"\n";
    }
    cv::Matx44d pose;double e;int n;
    require(!detect_pose(cv::Mat(720,1280,CV_8U,cv::Scalar(255)),K,spec,pose,e,n),"blank image accepted");
    Spec wrong=spec;wrong.dictionary=9;
    require(!detect_pose(image,K,wrong,pose,e,n),"wrong dictionary accepted");
    std::cout<<"CHARUCO_POSE_TEST_PASS\n";
}
