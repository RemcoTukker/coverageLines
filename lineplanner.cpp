#include "lineplanner.h"

using namespace cv;
using namespace std;

LinePlanner::LinePlanner()
{
    //ctor
}


void LinePlanner::loadMap(Mat &image, float wallFollowingRadius, float parallelDistance, float minLength)
//all distances in pixels
{
	map = image.clone();
	inflationPaths = Mat::zeros(image.size(), image.type());
    inflatedMap = Mat::zeros(image.size(), image.type());

	inflationRadius = wallFollowingRadius;

	//inflate the obstacles
	for(int i=0; i<image.rows; i++)
        for(int j=0; j<image.cols; j++)
            if (image.at<uchar>(i,j) < 250) inflationQueue.push(location {i,j,i,j} );

    while (!inflationQueue.empty()) processLocation(inflatedMap , inflationPaths, inflationRadius );


	///TODO: check if there are areas that are impossible to reach

	//do line detection
	vector<Vec4i> lines;
    HoughLinesP(inflationPaths, lines, 0.5, CV_PI/360, 30, 20, 200);


	///TODO: maybe some pathfinding to check whether the detected lines go through a wall. Also possible to do this
	/// with feedback from practice

    //for each line, propose parallel lines at a specific distance

    for( size_t i = 0; i < lines.size(); i++ )
    {
        int j = 0;
        while (true)
        {
            j++;
            if (addParallelLine(lines[i], j, minLength, parallelDistance ) == 1) break;
        }

        j = 0;
        while (true)
        {
            j--;
            if (addParallelLine(lines[i], j, minLength, parallelDistance) == 1) break;
        }
    }

}

void LinePlanner::visitLocation()
{

}

void LinePlanner::createPlan(float minCleanedFraction, float brushRadius)
{
	///TODO: check whether a map is loaded

	for(int i=0; i<inflationPaths.rows; i++)
        for(int j=0; j<inflationPaths.cols; j++)
            if (inflationPaths.at<uchar>(i,j) > 250 || inflatedMap.at<uchar>(i,j) > 250) inflationQueue.push(location {i,j,i,j} );
    cleanedArea = Mat::zeros(map.size(), map.type());
    Mat dummy = Mat::zeros(map.size(), map.type());
    while (!inflationQueue.empty()) processLocation(cleanedArea, dummy, 10 );

	long obstaclesAndWallFollowing = sum(cleanedArea)[0] / 255;
	long uncleanedArea = cleanedArea.rows * cleanedArea.cols - obstaclesAndWallFollowing;

    for (int l = 0; l < 1000; l++)
    {
        double cleaned = (sum(cleanedArea)[0] / 255.0 - obstaclesAndWallFollowing) / uncleanedArea;

        if (cleaned > minCleanedFraction) break;
		///TODO: stop if measure doesnt improve anymore with a certain delta


		vector<double> qualities;

        for( size_t i = 0; i < parallelLines.size(); i++ )
        {
            //
            Point start(parallelLines[i][0], parallelLines[i][1]);
            Point end(parallelLines[i][2], parallelLines[i][3]);
            double length = norm(start - end);
            Point normVector(parallelLines[i][1] - parallelLines[i][3], parallelLines[i][2] - parallelLines[i][0]);
            normVector = normVector * (brushRadius / length);

            double quality = 0;

            LineIterator it(cleanedArea, start, end, 8);
            for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
            {
                int alreadyCleaned = 0;
                int notYetCleaned = 0;

                LineIterator brush(cleanedArea, it.pos() - normVector, it.pos() + normVector, 8); //the position of the brush at a specific point
                for(int k = 0; k < brush.count; k++, ++brush)
                {
                    if (cleanedArea.at<uchar>(brush.pos()) > 0)
                        alreadyCleaned++;
                    else
                        notYetCleaned++;
                }
                quality += qualityFunction((double) notYetCleaned / ( (double) alreadyCleaned + notYetCleaned) );
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
        normVector = normVector * (brushRadius / length);

        LineIterator it(cleanedArea, start, end, 8);
        for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
        {
            line( cleanedArea, it.pos() - normVector, it.pos() + normVector, 255, 1, 8 );
        }

        //save this line
        bestLines.push_back(Vec4i {parallelLines[best][0], parallelLines[best][1],parallelLines[best][2], parallelLines[best][3]});

    }

}

bool LinePlanner::getNextLine(Point & start, Point & end)
{
	if (bestLines.empty()) return false;

	Vec4i bestLine = bestLines.back();
	bestLines.pop_back();
	start.x = bestLine[0];
	start.y = bestLine[1];
	end.x = bestLine[2];
	end.y = bestLine[3];

	return true;
}

void LinePlanner::processLocation(Mat& resultImage1, Mat& resultImage2, float radius)
{

    location currentLocation = inflationQueue.front();
    inflationQueue.pop();
    if (resultImage1.at<uchar>(currentLocation.x,currentLocation.y) > 0 ) return; //landed in already inflated space

    float distx = currentLocation.x - currentLocation.sourcex;
    float disty = currentLocation.y - currentLocation.sourcey;

    if (sqrt( distx*distx + disty*disty ) > radius )
    {

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

int LinePlanner::addParallelLine( Vec4i & line, int n, float minLength, float parallelDistance)
{

    Point shift( line[3] - line[1] , line[0] - line[2] ); //normal vector
	shift = shift * (parallelDistance / norm(shift) * n);
    Point start(line[0] + shift.x, line[1] + shift.y);
    Point end(line[2] + shift.x, line[3] + shift.y);

    // extend both endpoints
    extendEndpoint(start, end);
    extendEndpoint(end, start);

    //cout << "checked endpoints " << endl; cout.flush();

    ////third step: remove parts of the line that are inside an obstacle, split in two or more lines, remove parts < certain threshold
    //iterate over line until you hit an obstacle, save first part if large enough,
    //continue from the point where theres no obstacle anymore
    LineIterator it(inflatedMap, start, end, 8);

    bool currentlyInObstacle = true;
    Point lastPosition(start);
    int addedLines = 0;

    for(int j = 0; j < it.count; j++, ++it) //heh, kinda ugly
    {
        if ( inflatedMap.at<uchar>(it.pos()) != 0 && currentlyInObstacle == false )
            //just moved into obstacle or we're at the end of the line
        {
            currentlyInObstacle = true;
            if (norm(start - lastPosition) > 40)  ///TODO: hardcoded constant for minimum distance that a path has to be
            {
                parallelLines.push_back(Vec4i(start.x, start.y, lastPosition.x, lastPosition.y ));
                addedLines++;
            }

        }
        else if ( inflatedMap.at<uchar>(it.pos()) == 0 && currentlyInObstacle == true ) //just moved outside obstacle
        {
            currentlyInObstacle = false;
            start = it.pos();
        }

        lastPosition = it.pos();

    }

    if (addedLines == 0) return 1;

    return 0;
}

void LinePlanner::extendEndpoint(Point& start, Point& end)
{

    //cout << "extrapolating " << endl; cout.flush();
    int deltax = start.x - end.x;
    int deltay = start.y - end.y;
    Point newEndPoint(start);
    while(true) //extrapolate until outside image
    {
        newEndPoint += Point(deltax, deltay);
        if (newEndPoint.x < 0 || newEndPoint.y < 0 || newEndPoint.x > map.cols || newEndPoint.y > map.rows) break;
    }

    start = newEndPoint;

}

double LinePlanner::qualityFunction(double fraction) //pixels on brush that are not yet cleaned / total nr of pixels on brush
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

cv::Mat& LinePlanner::getCleanedArea()
{
	return cleanedArea;
}
