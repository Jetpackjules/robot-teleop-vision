#pragma once
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#if __has_include(<opencv2/objdetect/charuco_detector.hpp>)
#include <opencv2/objdetect/charuco_detector.hpp>
#define TELEOP_MODERN_CHARUCO
#else
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#endif
#include <cmath>
#include <string>
#include <vector>

namespace teleop_charuco {
struct Spec {
    int squares_x = 0, squares_y = 0;
    double square_m = 0, marker_m = 0;
    int dictionary = 0;
};
inline bool valid(const Spec &s) {
    return s.squares_x >= 3 && s.squares_x <= 40 && s.squares_y >= 3 && s.squares_y <= 40
        && std::isfinite(s.square_m) && std::isfinite(s.marker_m)
        && s.square_m >= .001 && s.square_m <= 1 && s.marker_m >= .0005 && s.marker_m < s.square_m
        && s.dictionary >= 0 && s.dictionary <= 15
        && s.squares_x*s.squares_y/2 <= (s.dictionary%4==0?50:s.dictionary%4==1?100:s.dictionary%4==2?250:1000);
}
inline bool detect_pose(const cv::Mat &image, const cv::Mat &K, const Spec &s,
                        cv::Matx44d &pose, double &error, int &count) {
    count = 0;
    if (!valid(s) || image.empty()) return false;
    cv::Mat gray;
    if (image.channels()==3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); else gray=image;
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    std::vector<cv::Point3f> board_points;
#ifdef TELEOP_MODERN_CHARUCO
    auto dictionary=cv::aruco::getPredefinedDictionary(s.dictionary);
    cv::aruco::CharucoBoard board(cv::Size(s.squares_x,s.squares_y),s.square_m,s.marker_m,dictionary);
    // This existing physical board uses the classic OpenCV board layout.
    board.setLegacyPattern(true);
    cv::aruco::CharucoParameters parameters;
    parameters.cameraMatrix=K;
    cv::aruco::CharucoDetector detector(board,parameters);
    detector.detectBoard(gray,corners,ids);
    board_points=board.getChessboardCorners();
#else
    auto dictionary=cv::aruco::getPredefinedDictionary(s.dictionary);
    auto board=cv::aruco::CharucoBoard::create(s.squares_x,s.squares_y,s.square_m,s.marker_m,dictionary);
    std::vector<std::vector<cv::Point2f>> markers;
    std::vector<int> marker_ids;
    // Marker corner subpixel refinement can be biased by adjacent chess squares.
    auto parameters=cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod=cv::aruco::CORNER_REFINE_NONE;
    cv::aruco::detectMarkers(gray,dictionary,markers,marker_ids,parameters);
    if(marker_ids.size()<3)return false;
    cv::aruco::interpolateCornersCharuco(markers,marker_ids,gray,board,corners,ids,K);
    board_points=board->chessboardCorners;
#endif
    if (corners.size()<8 || corners.size()!=ids.size())return false;
    std::vector<cv::Point3f> object;
    for(int id:ids){if(id<0||id>=int(board_points.size()))return false;object.push_back(board_points[id]);}
    std::vector<cv::Point2f> hull;
    cv::convexHull(corners,hull);
    if(std::abs(cv::contourArea(hull))<100)return false;
    std::vector<cv::Mat> rotations,translations;
    if(!cv::solvePnPGeneric(object,corners,K,cv::noArray(),rotations,translations,false,cv::SOLVEPNP_IPPE))return false;
    double best=1e9;
    for(size_t i=0;i<rotations.size();++i){
        if(translations[i].at<double>(2)<=0)continue;
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object,rotations[i],translations[i],K,cv::noArray(),projected);
        double sum=0,max_error=0;
        for(size_t j=0;j<corners.size();++j){double e=cv::norm(projected[j]-corners[j]);sum+=e*e;max_error=std::max(max_error,e);}
        double rms=std::sqrt(sum/corners.size());
        if(rms>=best||rms>1.0||max_error>2.5)continue;
        cv::Mat R;cv::Rodrigues(rotations[i],R);
        cv::Matx44d candidate=cv::Matx44d::eye();
        for(int row=0;row<3;++row){for(int col=0;col<3;++col)candidate(row,col)=R.at<double>(row,col);candidate(row,3)=translations[i].at<double>(row);}
        best=rms;pose=candidate;
    }
    if(best==1e9)return false;
    error=best;count=int(corners.size());return true;
}
}
