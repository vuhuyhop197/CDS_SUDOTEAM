#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace cv;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void update(const Mat &src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();

    static int slideThickness;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    static Point null; // 
    static double variance;
    static int LEFT_LANE;
    static int RIGHT_LANE;

private:
    Mat preProcess( const Mat &src, Point& mass_road );

    Mat birdViewTranform( const Mat &source, Point& mass_road );
    void fillLane( Mat &src );
    Mat sobelfilter( const Mat& img_gray);
    Point MassOfRoad(const Mat &src_RGB);
    
    void detectLeftRight( const vector<vector<Point> > &points, Point& mass_road );
    vector<Mat> splitLayer( const Mat &src, int dir = VERTICAL );
    vector<vector<Point> > centerRoadSide( const vector<Mat> &src, int dir = VERTICAL );
    int recognize_left_right( vector<Point>& lanex, Point& mass_road ); 

    bool point_in_rect( Rect rect_win, Point p);
    Point detectMassRoad( vector<vector<Point>>& contours );

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};
    int minLaneInShadow[3] = {90, 43, 97};
    int maxLaneInShadow[3] = {120, 80, 171};
    int binaryThreshold = 180;

    int skyLine = 85;
    int shadowParam = 40;

    vector<Point> leftLane, rightLane;
};

#endif
