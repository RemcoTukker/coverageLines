#include <iostream>
#include <opencv2/opencv.hpp>
#include "lineplanner.h"

using namespace std;
using namespace cv;


int main()
{
	Mat image;
    image = imread("map.pgm", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    cout << "image found " << image.type() << endl;

	LinePlanner planner;
	planner.loadMap(image, 12, 16, 40);

    cout << "map loaded "  << endl;

    planner.createPlan(0.98, 10);

    cout << "plan created " << endl;

    //start drawing
    Mat showPaths;
    cvtColor(image, showPaths, CV_GRAY2BGR );

    Point start, end;
    while ( planner.getNextLine(start, end) )
    {
    	line( showPaths, start, end, Scalar(0,255,0), 1, 8 );
    }

	Mat cleanedArea = planner.getCleanedArea();

    namedWindow( "Paths", 1 );
    imshow( "Paths", showPaths );
	namedWindow( "Cleaned", 1 );
    imshow( "Cleaned", cleanedArea );

    waitKey(0);


    return 0;
}

