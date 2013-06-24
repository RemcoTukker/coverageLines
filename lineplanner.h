#ifndef LINEPLANNER_H
#define LINEPLANNER_H

/*

 Code assumes a walled area as input!


 Copyright: Remco Tukker 2013


 TODO
 Segmentation! Take care of output of inflated wall following trajectories
 Diagonal lines do not fill up all pixel in the cleaned area, fix this!
 Maybe, use two different inflatedImages, one for obstacle detection, one for slightly
	further inflated wall following paths

*/


#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

struct location
{
    int x;
    int y;
    int sourcex;
    int sourcey;
};

class LinePlanner
{
    public:
        LinePlanner();
        void loadMap(cv::Mat &image, float wallFollowingRadius, float parallelDistance, float minLength);
        void visitLocation();
        void createPlan(float minCleanedFraction, float brushRadius);
        bool getNextLine(cv::Point & start, cv::Point & end);
        cv::Mat& getCleanedArea();
    protected:
	private:
		void processLocation(cv::Mat& resultImage1, cv::Mat& resultImage2, float radius);
        int addParallelLine( cv::Vec4i & line, int n, float minLength, float parallelDistance);
		void extendEndpoint(cv::Point& start, cv::Point& end);
		double qualityFunction(double fraction);
        std::queue<location> inflationQueue;
        cv::Mat map;
        cv::Mat inflatedMap;
        cv::Mat inflationPaths;
        cv::Mat cleanedArea;
        float inflationRadius;
        std::vector<cv::Vec4i> parallelLines;
        std::vector<cv::Vec4i> bestLines;
};

#endif // LINEPLANNER_H
