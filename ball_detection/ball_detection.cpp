#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <raspicam/raspicam_cv.h>

using namespace cv;
using namespace std;

//Ball measurements - units: mm(width) and pixels(focal length)
float width_ball_mm = 42.67;
int width_ball_pixels = 1, f = 980;
float min_distance = 30.0;

/*
LIGHT HSV
int low_H = 0, low_S = 88, low_V = 117;
int high_H = 7, high_S = 255, high_V = 255;

LESS LIGHT HSV
int low_H = 172, low_S = 70, low_V = 81;
int high_H = 179, high_S = 255, high_V = 252;
*/

int low_H = 0, low_S = 88, low_V = 117;
int high_H = 7, high_S = 255, high_V = 255;
const char* source_window = "Source";
const char* mask_window = "Mask";

VideoWriter writer("ball_detection_output.mp4", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(1280,960), true);

int main( int argc, char** argv )
{
    //VideoCapture cap(0);
    raspicam::RaspiCam_Cv camera;
    Mat src, frame, frame_HSV, frame_threshold;
    
    //set camera params. Image size = 960x1280 (h,w)
    int w = (2592/640)*320; //1280
    int h = (1992/480)*240; //960
    printf("image size %dx%d\n", h, w);
    
    camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    camera.set( CV_CAP_PROP_FRAME_HEIGHT, h);
    camera.set( CV_CAP_PROP_FRAME_WIDTH, w);
    
    //Open camera
	cout<<"Opening Camera..."<<endl;
	if (!camera.open()) {
        cerr<<"Error opening the camera"<<endl;
        return -1;
    }
    
    while(true){
		camera.grab();
        camera.retrieve (src);

        if(src.empty())
        {
            break;
        }
        Mat original_image = src.clone();
        
        // median filter and convert from BGR to HSV colorspace
        medianBlur(src, src, 11);
        cvtColor(src, frame_HSV, COLOR_BGR2HSV);
        
        // Detect the object based on HSV Range Values
        Mat kernel = getStructuringElement(MORPH_RECT, Size(11,11));
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        morphologyEx(frame_threshold, frame_threshold, MORPH_OPEN, kernel, Point(-1,-1), 2);        

        vector<vector<Point> > contours;
        findContours( frame_threshold, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

        /// Approximate contours to polygons + get bounding circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Point2f>centers( contours.size() );
        vector<float>radius( contours.size() );
        int idx = -1, max_area=0;
        bool ready_to_grab = false;
        
        for( size_t i = 0; i < contours.size(); i++ ){
            approxPolyDP( contours[i], contours_poly[i], 3, true );
            minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
            int area = (int)contourArea(contours_poly[i]);     
            if (area > max_area){
                idx = (int)i;
                max_area = area;
            }
        }
        
        Point_<int> v0,v1,h0,h1;
        int offset_ball = 145;
        h0 = Point(0,480 + offset_ball);
        h1 = Point(1280, 480 + offset_ball);
        v0 = Point(640 + offset_ball,0);
        v1 = Point(640 + offset_ball, 960);

        // Point in the image where we need to detect the ball in order to grab it correctly
        Point_<int> pt_ball_detection = Point(640 + offset_ball, 480 + offset_ball);
        circle( original_image, pt_ball_detection, 10, Scalar(0,0,255),-1);
        line(original_image, v0, v1, Scalar(0,0,255), 2);
        line(original_image, h0, h1, Scalar(0,0,255), 2);
        system("clear");

        if (idx>-1){
            circle( original_image, centers[idx], (int)radius[idx], Scalar(0,255,0), 2 );
            circle( original_image, centers[idx], 3, Scalar(255,0,0), -1 );

            //  Distance from the camera to the ball 
            width_ball_pixels = (int)radius[idx]*2;
            float distance = ((width_ball_mm/width_ball_pixels)*f)/10;
            printf("Distance to the ball: %f cm\n", distance);
  
            Point_<float> center_ball;
            center_ball = centers[idx];
            int diff_Vline = abs((int)center_ball.x - pt_ball_detection.x);

            if (distance > min_distance){
                printf("Go Straight\n");
                ready_to_grab = false;
            }
            if (diff_Vline > 20){ 
                if ((int)center_ball.x < pt_ball_detection.x){
                    printf("Turn left\n");
                    ready_to_grab = false;
                }
                if ((int)center_ball.x > pt_ball_detection.x){
                    printf("Turn right\n");
                    ready_to_grab = false;
                }
            }
            // The robot is ready to grab the ball if: distance to the ball < 30cm and distance to vertical line < 20 pixels
            if ( distance < min_distance && diff_Vline < 20){
                ready_to_grab = true;
            }
        }
        // Save the frame using the video writer
        writer.write(original_image);

        // Show the frame and the mask
        namedWindow( source_window );
        namedWindow( mask_window );
        imshow( source_window, original_image );
        imshow( mask_window, frame_threshold );

        // Save the image output
        Mat saved_image = original_image.clone();
        imwrite("raspicam_cv_image.png",saved_image);

        char key = (char) waitKey(10);
        if (key == 'q' || key == 27 || ready_to_grab == true){
            break;
        }
    }
    printf("The robot is ready to grab the ball\n");
    destroyAllWindows();
    writer.release();
    camera.release();
    return 0;
}
