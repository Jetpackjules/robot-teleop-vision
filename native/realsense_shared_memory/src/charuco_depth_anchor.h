#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
namespace teleop_charuco {
struct DepthAnchorStats { int samples=0; double rms_m=0, shift_m=0; };
inline bool anchor_depth_plane(const cv::Mat &depth,const cv::Vec4d &k,const cv::Matx44d &depth_to_color,
        const cv::Matx44d &visual,double width,double height,cv::Matx44d &anchored,DepthAnchorStats &stats) {
    if(depth.type()!=CV_32F||width<=0||height<=0)return false;
    cv::Matx33d R,E;cv::Vec3d t,et;
    for(int i=0;i<3;++i){t[i]=visual(i,3);et[i]=depth_to_color(i,3);for(int j=0;j<3;++j){R(i,j)=visual(i,j);E(i,j)=depth_to_color(i,j);}}
    std::vector<cv::Vec3d> points;
    for(int y=0;y<depth.rows;y+=2)for(int x=0;x<depth.cols;x+=2){
        double z=depth.at<float>(y,x);if(!std::isfinite(z)||z<=0)continue;
        cv::Vec3d p((x-k[2])*z/k[0],(y-k[3])*z/k[1],z);p=R.t()*(p-t);
        if(p[0]>.003&&p[0]<width-.003&&p[1]>.003&&p[1]<height-.003&&std::abs(p[2])<.03)points.push_back(p);
    }
    if(points.size()<200)return false;
    std::vector<double> z;for(auto p:points)z.push_back(p[2]);std::nth_element(z.begin(),z.begin()+z.size()/2,z.end());
    cv::Vec3d fit(0,0,z[z.size()/2]);std::vector<int> kept;
    for(int iteration=0;iteration<5;++iteration){
        kept.clear();double threshold=iteration==0?.004:.003;
        for(size_t i=0;i<points.size();++i)if(std::abs(points[i][2]-(fit[0]*points[i][0]+fit[1]*points[i][1]+fit[2]))<threshold)kept.push_back(int(i));
        if(kept.size()<200||kept.size()*5<points.size()*2)return false;
        cv::Mat A(int(kept.size()),3,CV_64F),b(int(kept.size()),1,CV_64F),out;
        for(size_t i=0;i<kept.size();++i){auto p=points[kept[i]];A.at<double>(int(i),0)=p[0];A.at<double>(int(i),1)=p[1];A.at<double>(int(i),2)=1;b.at<double>(int(i))=p[2];}
        if(!cv::solve(A,b,out,cv::DECOMP_SVD))return false;
        fit=cv::Vec3d(out.at<double>(0),out.at<double>(1),out.at<double>(2));
    }
    cv::Vec3d n=R*cv::Vec3d(-fit[0],-fit[1],1);n/=cv::norm(n);
    if(n.dot(R*cv::Vec3d(0,0,1))<std::cos(3*CV_PI/180))return false;
    cv::Vec3d plane=t+R*cv::Vec3d(0,0,fit[2]);double d=-n.dot(plane);cv::Vec3d origin=-(E.t()*et);
    auto intersection=[&](cv::Vec3d color,cv::Vec3d &point){
        if(color[2]<=0)return false;cv::Vec3d ray=E.t()*(color/color[2]);double denominator=n.dot(ray);
        if(std::abs(denominator)<.1)return false;double distance=-(n.dot(origin)+d)/denominator;
        if(!std::isfinite(distance)||distance<=0)return false;point=origin+ray*distance;return true;
    };
    cv::Vec3d q0,q1;
    if(!intersection(E*t+et,q0)||!intersection(E*(t+R*cv::Vec3d(.05,0,0))+et,q1))return false;
    cv::Vec3d x=q1-q0;if(cv::norm(x)<.001)return false;x/=cv::norm(x);cv::Vec3d y=n.cross(x);y/=cv::norm(y);x=y.cross(n);
    stats.shift_m=cv::norm(q0-t);if(stats.shift_m>.04)return false;
    double squared=0;for(int i:kept){auto p=points[i];double residual=p[2]-fit[0]*p[0]-fit[1]*p[1]-fit[2];squared+=residual*residual;}
    stats.samples=int(kept.size());stats.rms_m=std::sqrt(squared/kept.size());if(stats.rms_m>.003)return false;
    anchored=cv::Matx44d::eye();for(int i=0;i<3;++i){anchored(i,0)=x[i];anchored(i,1)=y[i];anchored(i,2)=n[i];anchored(i,3)=q0[i];}
    return true;
}
}
