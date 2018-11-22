#include "detectlane.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

int DetectLane::slideThickness = 10;
int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;
int DetectLane::VERTICAL = 0;
int DetectLane::HORIZONTAL = 1;
Point DetectLane::null = Point();
// Point DetectLane::mass_road_static = Point( BIRDVIEW_WIDTH/2, BIRDVIEW_HEIGHT/2 );
DetectLane::DetectLane() {
    createTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    createTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

    createTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    createTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    createTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    createTrackbar("HighV", "Threshold", &maxThreshold[2], 255);

    createTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
}

DetectLane::~DetectLane(){}

vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

vector<Point> DetectLane::getRightLane()
{
    return rightLane;
}

void DetectLane::update(const Mat &src)
{
    Point mass_road = null;
    Mat img = preProcess( src, mass_road );
    
    vector<Mat> layers1 = splitLayer(img);
    vector<vector<Point> > points1 = centerRoadSide(layers1);
    detectLeftRight( points1, mass_road );

    Mat birdView, lane;
    birdView = Mat::zeros(img.size(), CV_8UC3);
    lane = Mat::zeros(img.size(), CV_8UC3);

    for (int i = 0; i < points1.size(); i++)
     {
        for (int j = 0; j < points1[i].size(); j++)
        {
            circle(lane, points1[i][j], 1, Scalar(0,255,0), 2, 8, 0 );
        }
    }

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != null)
        {
            circle(lane, leftLane[i], 1, Scalar(155,155,50), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != null) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }
    circle( lane, mass_road, 1, Scalar(155,155,0), 2, 8, 0);
    imshow("Lane Detect", lane);
}

Mat DetectLane::preProcess(const Mat &src, Point& mass_road)
{
    Mat imgThresholded, imgHSV, imgsobel, dst;
    Point mass_road_temp = Point(0,0);

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), imgThresholded);

    imgsobel = sobelfilter( imgThresholded );
    Mat drawing_contours = MassOfRoad( src, mass_road_temp );

    // dst = birdViewTranform( thresh_contour, mass_road );
    dst = birdViewTranform( drawing_contours, mass_road_temp );
    fillLane(dst);

    imshow("Bird View", dst);
    imshow("Binary", imgThresholded);

    return dst;
}

Mat DetectLane::sobelfilter( const Mat& img_gray)
{
    int ddepth = CV_8U;
    int scale = 1;
    int delta = 0;
    Mat grad_x, grad_y, abs_grad_x, abs_grad_y, grad, thresh_grad;
    Sobel(img_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs( grad_x, abs_grad_x );
    Sobel(img_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    addWeighted( abs_grad_x, 0.2, abs_grad_y, 0.8, 0, grad );
    threshold(grad,thresh_grad, 80, 255,THRESH_BINARY);
    return thresh_grad;
}

void DetectLane::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 1);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, LINE_AA);
    }
}

vector<Mat> DetectLane::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    else 
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    
    return res;
}

vector<vector<Point> > DetectLane::centerRoadSide(const vector<Mat> &src, int dir)
{
    vector<std::vector<Point> > res;
    int inputN = src.size();
    for (int i = 0; i < inputN; i++) 
    {
        std::vector<std::vector<Point> > cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0) {
            res.push_back(tmp);
            continue;
        }
        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false);
            if (area > 3) {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                if (dir == VERTICAL) {
                    center1.y = center1.y + slideThickness*i;
                } 
                else
                {
                    center1.x = center1.x + slideThickness*i;
                }
                if (center1.x > 0 && center1.y > 0) {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }
    return res;
}

void DetectLane::detectLeftRight(const vector<vector<Point> > &points, Point& mass_road)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();
    
    leftLane.clear();
    rightLane.clear();

    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i ++)
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max1 = -1, max2 = -1;
    Point2i posMax1, posMax2;
    Rect top_rect( Point( 0, 0 ), Point( BIRDVIEW_WIDTH, 1) );
    Rect right_rect( Point( BIRDVIEW_WIDTH - 50, 0), Point(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT) );

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int k = 0; k < points[i + 1].size(); k ++)
            {
                if (abs(points[i + 1][k].x - points[i][j].x) < dis && 
                abs(points[i + 1][k].x - points[i][j].x) < err) 
                {
                    err = abs(points[i + 1][k].x - points[i][j].x);
                    pointMap[i][j] = pointMap[i + 1][k] + 1;
                    prePoint[i][j] = k;
                    postPoint[i + 1][k] = j;
                }
            }   
        }
    }
    // push_back lane point
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max1 && postPoint[i][j] == -1 
                & ~point_in_rect( top_rect, points[i].at(j) ) )
                // & ~point_in_rect( right_rect, points[i].at(j) ))
            {
                max1 = pointMap[i][j];
                posMax1 = Point2i(i, j);
            }
        }
    }
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax1.x || j != posMax1.y) && postPoint[i][j] == -1 
                & ~point_in_rect( top_rect, points[i].at(j) ) )
                // & ~point_in_rect( right_rect, points[i].at(j) ) )
            {
                
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }
    
    if (max1 == -1) return;
    while (max1 >= 1)
    {
        lane1.push_back(points[posMax1.x][posMax1.y]);
        if (max1 == 1) break;

        posMax1.y = prePoint[posMax1.x][posMax1.y];
        posMax1.x += 1;        
        
        max1--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;        
        
        max2--;
    }
    
    // detect left or right lane
    if ( recognize_left_right( lane1, mass_road ) )
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
    }
    else
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }     
    }

    if( max2 == - 1) return;
    if( recognize_left_right( lane2, mass_road ))
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }  
    }
    else 
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
        
    }
}
bool DetectLane::recognize_left_right( vector<Point>& lanex , Point& mass_road)
{
    vector<Point> subLane;
    for( size_t i = 0; i < lanex.size(); i++)
    {
        if( lanex.at(i) != null )
        {
            subLane.push_back( lanex.at(i) );
        }
    }
    Vec4f line;
    fitLine(subLane, line, DIST_L1, 0, 0.01, 0.01);
    float y_mass = (float)( mass_road.y );
    float x_mass = (float)( mass_road.x );
    // x_ref = Vx/Vy * (y_point - yo) + xo
    float x_line = line[0]/line[1]*( y_mass - line[3] ) + line[2];
    // true is left lane
    return ( x_line <= x_mass )? true : false;
}
bool DetectLane::point_in_rect( Rect rect_win, Point p)
{
    int x_min = rect_win.tl().x;
    int y_min = rect_win.tl().y;
    int x_max = rect_win.br().x;
    int y_max = rect_win.br().y;
    bool in_rect = ( p.x >= x_min );
    in_rect = in_rect & ( p.x <= x_max );
    in_rect = in_rect & ( p.y >= y_min );
    in_rect = in_rect & ( p.y <= y_max );
    return in_rect;
}
Mat DetectLane::birdViewTranform( const Mat &src, Point& mass_road)
{
    Point2f src_vertices[4];
    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, skyLine);
    src_vertices[1] = Point(width, skyLine);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 90, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(90, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
    // tranform massroad
    vector<Point2f> dstPoints, srcPoints;
    srcPoints.push_back( Point2f( mass_road ) );
    perspectiveTransform( srcPoints, dstPoints, M );
    mass_road = Point( dstPoints.at(0).x, dstPoints.at(0).y - skyLine );
    return dst;
}

Mat DetectLane::MassOfRoad(const Mat &src_RGB, Point &mass_road)
{
    Scalar HSV_thresh_min = Scalar(0,1,101);
    Scalar HSV_thresh_max = Scalar(255,255,255);
    Mat src_HSV, road_thresh, road_thresh_inv;
    Point center = Point(0,0);

    int offset_y = 80;
    Rect roi;
    roi.x = 0;
    roi.y = offset_y;
    roi.width = src_RGB.size().width;
    roi.height = src_RGB.size().height - offset_y;
    Mat src_crop = src_RGB(roi);

    vector<Point> max_contours;
    double max_area = 0;
    int max_index = 0;
    // Vec4i max_hierarchy = Vec4i(0, 0, 0, 0);

    cvtColor(src_crop, src_HSV, COLOR_BGR2HSV);
    inRange(src_HSV, HSV_thresh_min, HSV_thresh_max, road_thresh);
    bitwise_not(road_thresh,road_thresh_inv);

    Mat thresh_fullSize = thresh_fullSize.zeros(Size (320,240), src_RGB.type());
    copyMakeBorder(road_thresh_inv, thresh_fullSize, offset_y, 0, 0, 0, BORDER_CONSTANT, Scalar(0, 0, 0));
    /// Find contours
    vector<vector<Point> > contours; //Tao 1 vector 2 chieu voi moi phan tu la Point
    vector<Vec4i> hierarchy;
    findContours( thresh_fullSize, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    for ( size_t i = 0 ; i < contours.size() ; i++ )
    {
        double area = fabs( contourArea( contours[i] ) );
        if (  area > max_area )
        {
            max_area  = area;
            max_index = i;
            max_contours = contours[max_index];
        }
    }

    Moments mu = moments( max_contours, false );
    Point mc = Point( mu.m10/mu.m00 , mu.m01/mu.m00 );

    Mat drawing = Mat::zeros( thresh_fullSize.size(), CV_8UC1 );
    drawShapes(drawing, max_contours);
    // const Point* ppt[1] = {max_contours.begin()};

    // polylines(drawing, ppt, max_contours.size(), 1, true, Scalar(255,255,255), 5, 8);

    // drawing = cvApproxPoly(max_contours, max_area, contours, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
    // for( int i = 0; i< contours.size(); i++ )
    // {
    // drawContours( drawing, max_contours, max_index, Scalar( 255, 255, 255), 2, 8, hierarchy, 0, Point() );
    circle( drawing, mc, 1, Scalar (255,255,255), -1, 8, 0 );
    // }
    imshow("Debug", drawing);

    return drawing;
}

void DetectLane::drawShapes( Mat& image, const vector<Point> & Shapes)
{
    int n_point = Shapes.size();
    Point_<int> p;
    const Point* a = &p;
    
    p = Point_<int>(0,0);
    for(size_t j = 0; j < Shapes.size(); j++)
    {
        p += Shapes[j];
    }
    //(int)Shapes[i].size();
    //putText(image, nameshapes, p[0],FONT_HERSHEY_COMPLEX_SMALL, 1, color, 1, LINE_AA);
    p = p/int(Shapes.size());
    // cout << Shapes.size() <<endl;
    polylines(image, &a, &n_point, 1, true, Scalar(255,255,255), 5, FILLED );

}
