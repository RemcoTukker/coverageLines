#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

using namespace std;
using namespace cv;


struct location
{
    int x;
    int y;
    int sourcex;
    int sourcey;
};

queue<location> inflationQueue;
Mat image;
Mat inflatedImage;
Mat inflationPaths;
float radius = 8;
vector<Vec4i> parallelLines;

void processLocation()
{


    location currentLocation = inflationQueue.front();
    inflationQueue.pop();
    if (inflatedImage.at<uchar>(currentLocation.x,currentLocation.y) > 0 ) return; //landed in already inflated space

    float distx = currentLocation.x - currentLocation.sourcex;
    float disty = currentLocation.y - currentLocation.sourcey;

    if (sqrt( distx*distx + disty*disty ) > radius )
    {
        //TODO: check if this position is in collision with something
        //shouldnt be necessary, as we are inflating from all obstacles simultaneously
        ///however, beware border cases where obstacles are just outside the map! Include a half meter band around the area
        ///that should be cleaned! Can easily be done by building a 1 pixel fake wall around selection
        ///Thus, the code assumes a walled area as input!

        inflationPaths.at<uchar>(currentLocation.x, currentLocation.y) = 255;
        return;

    }

    inflatedImage.at<uchar>(currentLocation.x,currentLocation.y) = 255; //black means already visited

    //add new positions in the queue
    if (currentLocation.x < inflatedImage.rows - 1)
        inflationQueue.push(location {currentLocation.x + 1, currentLocation.y, currentLocation.sourcex, currentLocation.sourcey} );
    if (currentLocation.x > 0)
        inflationQueue.push(location {currentLocation.x - 1, currentLocation.y, currentLocation.sourcex, currentLocation.sourcey} );
    if (currentLocation.y < inflatedImage.cols - 1)
        inflationQueue.push(location {currentLocation.x, currentLocation.y + 1, currentLocation.sourcex, currentLocation.sourcey} );
    if (currentLocation.y > 0)
        inflationQueue.push(location {currentLocation.x, currentLocation.y - 1, currentLocation.sourcex, currentLocation.sourcey} );

    return;
}

void checkEndpoint(Point& start, Point& end)
{

    //first: extend line to two directions in case endpoint is not in obstacle or outside picture
    //extrapolate until outside image

    //iterate over line until obstacle from original endpoints

    //save new start and endpoint

    //second: clip line at both ends in case in obstacle or outside image
    //iterate over line from both sides and save first point outside obstacle

    if (inflatedImage.at<uchar>(start.x, start.y) == 0 ) //extrapolate until obstacle
    {
        cout << "extrapolating " << endl; cout.flush();
        int deltax = start.x - end.x;
        int deltay = start.y - end.y;
        Point newEndPoint(start);
        while(true) //first extrapolate until outside image
        {
            newEndPoint += Point(deltax, deltay);
            if (newEndPoint.x < 0 || newEndPoint.y < 0 || newEndPoint.x > image.cols || newEndPoint.y > image.rows) break;
        }

        LineIterator it(inflatedImage, start, newEndPoint, 8);
        for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
        {
            if ( inflatedImage.at<uchar>(it.pos()) != 0 ) break; //stop as soon as we hit an obstacle
            newEndPoint = it.pos(); //we need the point one back on breaking, not the current one
        }
        start = newEndPoint;
    }
    else //clip line until outside obstacle
    {
        cout << "clipping " << endl; cout.flush();
        LineIterator it(inflatedImage, start, end, 8); //check if something is going wrong here..

        for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
        {
            cout << it.pos() << endl; cout.flush();
            if ( inflatedImage.at<uchar>(it.pos()) == 0 ) break; //stop as soon as were in free space
        }
        start = it.pos();
    }
}

int addParallelLine(Point& start, Point& end)
{
    ///TODO, maybe, use two different inflatedImages, one for obstacle detection, one for slightly
    /// further inflated wall following paths

    //clip or extend both endpoints
    checkEndpoint(start, end);
    checkEndpoint(end, start);

    cout << "checked endpoints " << endl; cout.flush();

    ////third step: remove parts of the line that are inside an obstacle, split in two or more lines, remove parts < certain threshold
    //iterate over line until you hit an obstacle, save first part if large enough,
    //continue from the point where theres no obstacle anymore
    LineIterator it(inflatedImage, start, end, 8);

    bool currentlyInObstacle = false;
    Point lastPosition(start);

    for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
    {
        if ( (inflatedImage.at<uchar>(it.pos()) != 0 && currentlyInObstacle == false) || j + 1 == it.count )
            //just moved into obstacle or we're at the end of the line
        {
            currentlyInObstacle = true;
            if (norm(start - lastPosition) > 40)  ///TODO: hardcoded constant for minimum distance that a path has to be
            {
                parallelLines.push_back(Vec4i(start.x, start.y, lastPosition.x, lastPosition.y ));
            }

        }
        else if ( inflatedImage.at<uchar>(it.pos()) == 0 && currentlyInObstacle == true ) //just moved outside obstacle
        {
            currentlyInObstacle = false;
            start = it.pos();
        }

        lastPosition = it.pos();

    }
    return 0;
}

int main()
{

    image = imread("sligro_up.pgm", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    cout << "image loaded " << image.type() << endl;

    inflationPaths = Mat::zeros(image.size(), image.type());
    inflatedImage = Mat::zeros(image.size(), image.type());

    //namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", image );                   // Show our image inside it.

    //inflate obstacles

    //add obstacles
    for(int i=0; i<image.rows; i++)
        for(int j=0; j<image.cols; j++)
            if (image.at<uchar>(i,j) < 250) inflationQueue.push(location {i,j,i,j} );

    while (!inflationQueue.empty()) processLocation();

    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", inflatedImage );                   // Show our image inside it.
    namedWindow( "Display window2", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window2", inflationPaths );                   // Show our image inside it.
    // waitKey(0);


    //check each position on collisions, using a slighly smaller inflation radius, remove points that are in collision
    //shouldnt be necessary.. after all, we did the inflation...

    //now do line detection
    Mat linePaths = inflationPaths.clone();
    Mat showPaths;
    cvtColor(linePaths, showPaths, CV_GRAY2BGR );

    vector<Vec4i> lines;
    HoughLinesP(linePaths, lines, 0.5, CV_PI/360, 30, 20, 200);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( showPaths, Point(lines[i][0], lines[i][1]),
              Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
    }

    ///TODO: maybe some pathfinding to check whether the detected lines go through a wall. Also possible to do this with feedback from
    /// practice

    /*
        vector<Vec2f> lines;
        HoughLines( linePaths, lines, 2, CV_PI/180, 100 );

        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( showPaths, pt1, pt2, Scalar(0,0,255), 3, 8 );
        }
    */



    //for each line, propose parallel lines at a specific distance

    for( size_t i = 0; i < lines.size(); i++ )
    {
        int j = 0;
        while (true)
        {
            j++;
            Point shift( lines[i][3] - lines[i][1] , lines[i][0] - lines[i][2] ); //normal vector
            shift = shift * (50.0 / norm(shift) * j); ///TODO: hardcoded distance number here, replace!
//
//            if (lines[i][0] + shift.x < 0 || lines[i][0] + shift.x > image.cols ||
//                lines[i][2] + shift.x < 0 || lines[i][2] + shift.x > image.cols ||
//                lines[i][1] + shift.y < 0 || lines[i][1] + shift.y > image.rows ||
//                lines[i][3] + shift.y < 0 || lines[i][3] + shift.y > image.rows ) break;
//            parallelLines.push_back(Vec4i(lines[i][0] + shift.x, lines[i][1] + shift.y,
//                                          lines[i][2] + shift.x, lines[i][3] + shift.y ) );

            Point start(lines[i][0] + shift.x, lines[i][1] + shift.y);
            Point end(lines[i][2] + shift.x, lines[i][3] + shift.y);

            cout << start << " " << end << endl;
            cout.flush();

            if (addParallelLine(start, end ) == 1) break;

        }

        j = 0;
        while (true)
        {
            j++;
            Point shift( lines[i][1] - lines[i][3] , lines[i][2] - lines[i][0] ); //other normal vector
            shift = shift * (50.0 / norm(shift) * j); ///TODO: hardcoded distance number here, replace!

            Point start(lines[i][0] + shift.x, lines[i][1] + shift.y);
            Point end(lines[i][2] + shift.x, lines[i][3] + shift.y);

            if (addParallelLine(start, end ) == 1) break;
        }

    }

    //remove lines that have a substantial part inside obstacles
//    for( size_t i = 0; i < parallelLines.size(); i++ )
//    {
//        LineIterator it(inflatedImage, Point(parallelLines[i][0], parallelLines[i][1]),
//            Point(parallelLines[i][2], parallelLines[i][3]), 8);
//
//        int obstaclePoints = 0;
//        for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
//        {
//            if ( inflatedImage.at<uchar>(it.pos()) != 0 )
//            {
//                obstaclePoints++;
//                if (obstaclePoints > 20)
//                {
//                    ///TODO: handle more intelligently.. for now, just throw away
//                    parallelLines.erase(parallelLines.begin() + i);
//                    i--; //make sure we dont skip elements
//                    break;
//                }
//            }
//            else
//            {
//                obstaclePoints = 0;
//            }
//        }
//
//
//
//    }



    for( size_t i = 0; i < parallelLines.size(); i++ )
    {
        line( showPaths, Point(parallelLines[i][0], parallelLines[i][1]),
              Point(parallelLines[i][2], parallelLines[i][3]), Scalar(0,255,0), 1, 8 );
    }


    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", showPaths );

    waitKey(0);


    //calculate the amount of overlap (with the inflationPaths) at each point on each line and process this to a quality factor
    //another possibility: calculate whether there will be any small closed areas left if this path is included

    //select best path and repeat with paths so far till 95% of area is covered or something like that

    return 0;
}

