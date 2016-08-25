#include "stdio.h"
#include "math.h"
#include "opencv/cv.h"
#include <opencv/highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  VideoCapture cap(0); //-1, 0, 1 device id
  Mat flow, frame;
  // some faster than mat image container
  Mat flowUmat, prevgray, eig_image, temp_image;
  Mat img;
  Mat original;
  Mat pyramid1, pyramid2;

  CvSize frame_size;
  frame_size.height = 480;
  frame_size.width = 640;

  while(1)
  {
    bool Is = cap.grab();
    if (Is == false) {
      cout << "Video Capture Fail" << endl;
      break;
    }
    // capture frame from video file
    cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
    resize(img, img, Size(640, 480));
    // save original for later
    img.copyTo(original);
    // just make current frame gray
    cvtColor(img, img, COLOR_BGR2GRAY);
    // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous                           //and current frame
    //if there is no current frame
    // go to this part and fill previous frame
    //else {
    // img.copyTo(prevgray);
    //   }
    // if previous frame is not empty.. There is a picture of previous frame. Do some                                  //optical flow alg.
    if (prevgray.empty() == false ) {
      int number_of_features = 400;
      vector<Point2f> features;
      vector<Point2f> features_after;
      vector<uchar> status;
      vector<float> err;
      int maxCount = 50;
      double minDis = 20;
      double qLevel = 0.01;

      goodFeaturesToTrack(prevgray, features, maxCount, qLevel, minDis);

      // calculate optical flow
      //calcOpticalFlowPyrLK(prevgray, img, frame1_features, frame2_features, number_of_features, flow_found_feature, flow_feature_error);
      calcOpticalFlowPyrLK(prevgray, img, features, features_after, status, err);

      int k = 0;
      for(int i=0;i<features_after.size();i++)
      {
        //状态要是1，并且坐标要移动下的那些点
        if(status[i]&&((abs(features[i].x-features_after[i].x)+
        abs(features[i].y-features_after[i].y))>4))
        {
          features_after[k++] = features_after[i];
        }
      }
      features_after.resize(k);

      for(int i=0;i<features_after.size();i++)
      {
        circle(prevgray, features_after[i], 3, Scalar(255), 2);
      }

      // draw the results
      namedWindow("prew", WINDOW_AUTOSIZE);
      imshow("prew", prevgray);

      // fill previous image again
      img.copyTo(prevgray);
    }
    else {
      // fill previous image in case prevgray.empty() == true
      img.copyTo(prevgray);
    }

    char c = waitKey(3);
    if (c == 27) break;
  }

  return 0;
}
