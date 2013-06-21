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

vector<Vec4i> parallelLines;

void processLocation(Mat& resultImage1, Mat& resultImage2, float radius)
{


    location currentLocation = inflationQueue.front();
    inflationQueue.pop();
    if (resultImage1.at<uchar>(currentLocation.x,currentLocation.y) > 0 ) return; //landed in already inflated space

    float distx = currentLocation.x - currentLocation.sourcex;
    float disty = currentLocation.y - currentLocation.sourcey;

    if (sqrt( distx*distx + disty*disty ) > radius )
    {
        //TODO: check if this position is in collision with something
        //shouldnt be necessary, as we are inflating from all obstacles simultaneously
        ///however, beware border cases where obstacles are just outside the map! Include a half meter band around the area
        ///that should be cleaned! Can easily be done by building a 1 pixel fake wall around selection
        ///Thus, the code assumes a walled area as input!

        resultImage2.at<uchar>(currentLocation.x, currentLocation.y) = 255;
        return;

    }

    resultImage1.at<uchar>(currentLocation.x,currentLocation.y) = 255; //white means already visited

    //add new positions in the queue
    if (currentLocation.x < resultImage1.rows - 1)
        inflationQueue.push(location {currentLocation.x + 1, currentLocation.y, currentLocation.sourcex, currentLocation.sourcey} );
    if (currentLocation.x > 0)
        inflationQueue.push(location {currentLocation.x - 1, currentLocation.y, currentLocation.sourcex, currentLocation.sourcey} );
    if (currentLocation.y < resultImage1.cols - 1)
        inflationQueue.push(location {currentLocation.x, currentLocation.y + 1, currentLocation.sourcex, currentLocation.sourcey} );
    if (currentLocation.y > 0)
        inflationQueue.push(location {currentLocation.x, currentLocation.y - 1, currentLocation.sourcex, currentLocation.sourcey} );

    return;
}

double qualityFunction(double fraction) //pixels on brush that are not yet cleaned / total nr of pixels on brush
{
    if ( fraction >= 1)
        return 0.3;
    else if ( fraction >= 0.8)
        return 1;
    else if ( fraction >= 0.5)
        return 0.5;
    else if ( fraction >= 0.2)
        return 0.3;
    else if (fraction > 0)
        return 0.1;
    else
        return -0.5;
}

void extendEndpoint(Point& start, Point& end)
{

    //cout << "extrapolating " << endl; cout.flush();
    int deltax = start.x - end.x;
    int deltay = start.y - end.y;
    Point newEndPoint(start);
    while(true) //extrapolate until outside image
    {
        newEndPoint += Point(deltax, deltay);
        if (newEndPoint.x < 0 || newEndPoint.y < 0 || newEndPoint.x > image.cols || newEndPoint.y > image.rows) break;
    }

    start = newEndPoint;

}

int addParallelLine(Point& start, Point& end)
{
    ///TODO, maybe, use two different inflatedImages, one for obstacle detection, one for slightly
    /// further inflated wall following paths

    // extend both endpoints
    extendEndpoint(start, end);
    extendEndpoint(end, start);

    //cout << "checked endpoints " << endl; cout.flush();

    ////third step: remove parts of the line that are inside an obstacle, split in two or more lines, remove parts < certain threshold
    //iterate over line until you hit an obstacle, save first part if large enough,
    //continue from the point where theres no obstacle anymore
    LineIterator it(inflatedImage, start, end, 8);

    bool currentlyInObstacle = true;
    Point lastPosition(start);
    int addedLines = 0;

    for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
    {
        if ( inflatedImage.at<uchar>(it.pos()) != 0 && currentlyInObstacle == false )
            //just moved into obstacle or we're at the end of the line
        {
            currentlyInObstacle = true;
            if (norm(start - lastPosition) > 40)  ///TODO: hardcoded constant for minimum distance that a path has to be
            {
                parallelLines.push_back(Vec4i(start.x, start.y, lastPosition.x, lastPosition.y ));
                addedLines++;
            }

        }
        else if ( inflatedImage.at<uchar>(it.pos()) == 0 && currentlyInObstacle == true ) //just moved outside obstacle
        {
            currentlyInObstacle = false;
            start = it.pos();
        }

        lastPosition = it.pos();

    }

    if (addedLines == 0) return 1;

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


    //inflate obstacles

    //add obstacles
    for(int i=0; i<image.rows; i++)
        for(int j=0; j<image.cols; j++)
            if (image.at<uchar>(i,j) < 250) inflationQueue.push(location {i,j,i,j} );

    while (!inflationQueue.empty()) processLocation(inflatedImage , inflationPaths, 12 ); ///TODO: hardcoded inflation radius!

    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", inflatedImage );                   // Show our image inside it.
    namedWindow( "Display window2", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window2", inflationPaths );                   // Show our image inside it.
    // waitKey(0);



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

    //for each line, propose parallel lines at a specific distance

    for( size_t i = 0; i < lines.size(); i++ )
    {
        int j = 0;
        while (true)
        {
            j++;
            Point shift( lines[i][3] - lines[i][1] , lines[i][0] - lines[i][2] ); //normal vector
            shift = shift * (16.0 / norm(shift) * j); ///TODO: hardcoded distance number here, replace!

            Point start(lines[i][0] + shift.x, lines[i][1] + shift.y);
            Point end(lines[i][2] + shift.x, lines[i][3] + shift.y);

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

    cout << parallelLines.size() << endl;
    cout.flush();


    for( size_t i = 0; i < parallelLines.size(); i++ )
    {
        line( showPaths, Point(parallelLines[i][0], parallelLines[i][1]),
              Point(parallelLines[i][2], parallelLines[i][3]), Scalar(0,255,0), 1, 8 );
    }


    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", showPaths );




    //inflate inflationpaths to show cleaned area, superimpose obstacles?
    for(int i=0; i<inflationPaths.rows; i++)
        for(int j=0; j<inflationPaths.cols; j++)
            if (inflationPaths.at<uchar>(i,j) > 250) inflationQueue.push(location {i,j,i,j} );
    Mat cleanedArea = Mat::zeros(image.size(), image.type());
    Mat dummy = Mat::zeros(image.size(), image.type());
    while (!inflationQueue.empty()) processLocation(cleanedArea, dummy, 10 );





    vector<Vec4i> bestLines;

    for (int l = 0; l < 1000; l++)
    {
        double cleaned = sum(cleanedArea)[0] / 255 / (cleanedArea.rows * cleanedArea.cols);
        cout << cleaned << endl;
        cout.flush();

        if (cleaned > 0.63) break;


        vector<double> qualities;

        for( size_t i = 0; i < parallelLines.size(); i++ )
        {
            //
            Point start(parallelLines[i][0], parallelLines[i][1]);
            Point end(parallelLines[i][2], parallelLines[i][3]);
            double length = norm(start - end);
            Point normVector(parallelLines[i][1] - parallelLines[i][3], parallelLines[i][2] - parallelLines[i][0]);
            normVector = normVector * (10.0 / length); ///TODO: hardcoded brush radius here!

            double quality = 0;

            LineIterator it(cleanedArea, start, end, 8);
            for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
            {
                int alreadyCleaned = 0;
                int notYetCleaned = 0;

                LineIterator brush(cleanedArea, it.pos() - normVector, it.pos() + normVector, 8); //the position of the brush at a specific point
                for(int k = 0; k < brush.count; k++, ++brush) ///TODO: does this even work?
                {
                    if (cleanedArea.at<uchar>(it.pos()) > 0)
                        alreadyCleaned++;
                    else
                        notYetCleaned++;
                }
                quality += qualityFunction(notYetCleaned / (alreadyCleaned + notYetCleaned) );
            }

            //calculate score   (pixels on line that have nice normals - pixels on line that dont have nice normals)
            // or just (sum of niceness of pixels on line)
            //calculate the amount of overlap (with the inflationPaths) at each point on each line and process this to a quality factor
            //another possibility: calculate whether there will be any small closed areas left if this path is included

            //save quality
            qualities.push_back(quality);
        }




        //select best path and repeat with paths so far till 95% of area is covered or something like that
        double highestQuality = -10000;
        int best = -1;
        for( size_t i = 0; i < parallelLines.size(); i++ )
        {
            if (qualities[i] > highestQuality)
            {
                highestQuality = qualities[i];
                best = i;
            }
        }



        Point start(parallelLines[best][0], parallelLines[best][1]);
        Point end(parallelLines[best][2], parallelLines[best][3]);
        double length = norm(start - end);
        Point normVector(parallelLines[best][1] - parallelLines[best][3], parallelLines[best][2] - parallelLines[best][0]);
        normVector = normVector * (10.0 / length); ///TODO: hardcoded brush radius here!

        LineIterator it(cleanedArea, start, end, 8);
        for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
        {
            line( cleanedArea, it.pos() - normVector, it.pos() + normVector, 255, 1, 8 );
            //    LineIterator brush(cleanedArea, it.pos() - normVector, it.pos() + normVector, 8); //the position of the brush at a specific point
            //    for(int k = 0; k < brush.count; k++, ++brush)
            //    {
            //        cleanedArea.at<uchar>(brush.pos()) = 255;
            //    }
            }

//        Mat bestPath = Mat::zeros(image.size(), image.type());
//        line( bestPath, Point(parallelLines[best][0], parallelLines[best][1]),
//              Point(parallelLines[best][2], parallelLines[best][3]), 255, 1, 8 );
//        for(int k=0; k<bestPath.rows; k++)
//            for(int j=0; j<bestPath.cols; j++)
//                if (bestPath.at<uchar>(k,j) > 250) inflationQueue.push(location {k,j,k,j} );
//        while (!inflationQueue.empty()) processLocation(bestPath, dummy, 10 );
//
//        //merge bestPath into cleanedArea
//        bitwise_or(cleanedArea, bestPath, cleanedArea );
//
        //save this line for a graphical representation
        bestLines.push_back(Vec4i {parallelLines[best][0], parallelLines[best][1],parallelLines[best][2], parallelLines[best][3]});

    }

    Mat bestPaths = Mat::zeros(showPaths.size(), showPaths.type());
    for( size_t i = 0; i < bestLines.size(); i++ )
    {
        line( bestPaths, Point(bestLines[i][0], bestLines[i][1]),
              Point(bestLines[i][2], bestLines[i][3]), Scalar(0,255,0), 1, 8 );
    }



    namedWindow( "Cleaned Area", 1 );
    imshow( "Cleaned Area", cleanedArea );
    namedWindow( "Best Path", 1 );
    imshow( "Best Path", bestPaths );

    waitKey(0);


    return 0;
}

