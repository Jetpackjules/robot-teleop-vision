#include "charuco_depth_anchor.h"
#include <iostream>
#include <stdexcept>
void require(bool b,const char *m){if(!b)throw std::runtime_error(m);}
int main(){
 cv::Vec4d k(500,500,320,240);cv::Matx44d pose=cv::Matx44d::eye();pose(0,3)=-.12;pose(1,3)=-.08;pose(2,3)=.7;
 for(double bias:{-.0085,0.,.012})for(double slope:{0.,.015}){
  cv::Mat depth(480,640,CV_32F);for(int y=0;y<480;++y)for(int x=0;x<640;++x)depth.at<float>(y,x)=float((.7+bias)/(1-slope*(x-320)/500.));
  cv::Matx44d out;teleop_charuco::DepthAnchorStats stats;
  require(teleop_charuco::anchor_depth_plane(depth,k,cv::Matx44d::eye(),pose,.288,.18,out,stats),"valid depth plane rejected");
  cv::Vec3d origin(out(0,3),out(1,3),out(2,3));require(std::abs(origin[2]-slope*origin[0]-(.7+bias))<.00001,"depth offset not corrected");
  cv::Vec3d normal(-slope,0,1);normal/=cv::norm(normal);require(cv::norm(normal-cv::Vec3d(out(0,2),out(1,2),out(2,2)))<.0001,"depth tilt not corrected");
  require(stats.samples>=200&&stats.rms_m<.0001,"plane precision invalid");
 }
 cv::Mat blank(480,640,CV_32F,cv::Scalar(0));cv::Matx44d out;teleop_charuco::DepthAnchorStats stats;require(!teleop_charuco::anchor_depth_plane(blank,k,cv::Matx44d::eye(),pose,.288,.18,out,stats),"missing depth accepted");
 std::cout<<"AUTOMATIC_DEPTH_ANCHOR_PASS: positive/negative bias, tilt, missing depth\n";
}
