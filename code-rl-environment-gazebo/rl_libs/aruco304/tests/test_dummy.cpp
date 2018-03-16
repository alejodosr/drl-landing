#include "aruco.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace cv;
using namespace aruco;
using namespace std;

int main()
{
    VideoCapture cap;
    cap.open(0);
    Mat frame;
    Mat InImage;
    MarkerDetector MDetector;
    vector<Marker> Markers;

    MDetector.setDictionary(Dictionary::ARUCO_MIP_36h12);
    bool end=false;
    while( !end)
    {
        cap  >> frame;
        cvtColor(frame, InImage, COLOR_BGR2GRAY);
        Markers = MDetector.detect(InImage);
        for (size_t i = 0; i < Markers.size(); i++)
        {
            cout << Markers[i] << endl;
            Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
        }
        namedWindow("in", 1);
        imshow("in", InImage);
        char k=waitKey(30);
        if(k==27) end=true;
    }
}


