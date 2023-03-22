#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "NoGPS.h"

std::vector<cv::Point2f> obtainCorners(cv::Mat dst);
void mouseCallback(int event, int x, int y, int flags, void* userData);

std::vector<Point2D> myMap{};
Point2D lostPoint{};

constexpr auto WINDOW = "Localization";

int main() {
	auto img = cv::imread("input/hcmut_campus.jpg");

	cv::Mat gray;
	cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	gray.convertTo(gray, CV_32FC1);

	cv::Mat dst = cv::Mat::zeros(gray.size(), CV_32FC1);
	cornerHarris(gray, dst, 2, 3, 0.04);
	//result is dilated for marking the corners, not important
	dilate(dst, dst, cv::Mat(), cv::Point(-1, -1));

    // Initialize my map
    const auto corners = obtainCorners(dst);
	for (const auto& corner : corners) {
        myMap.emplace_back(corner.x, corner.y);
    }


    // Threshold for an optimal value, it may vary depending on the image.
    for (int i = 0; i < dst.rows; i++) {
        for (int j = 0; j < dst.cols; j++) {
            if (dst.at<float>(i, j) > 0.01f * dst.at<float>(0, 0)) {
                img.at<cv::Vec3b>(i, j)[0] = 0;
                img.at<cv::Vec3b>(i, j)[1] = 0;
                img.at<cv::Vec3b>(i, j)[2] = 255;
            }
        }
    }


    std::cout << "==================== NO GPS LOCALIZATION ====================== \n";
    cv::namedWindow(WINDOW);
    setMouseCallback(WINDOW, mouseCallback, &img);
    imshow(WINDOW, img);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}


std::vector<cv::Point2f> obtainCorners(cv::Mat dst) {
    double minVal, maxVal;
    minMaxLoc(dst, &minVal, &maxVal);

    const auto thresh = 0.01 * maxVal;
    threshold(dst, dst, thresh, 255, 0);
    dst.convertTo(dst, CV_8UC1);

    // find centroids
    cv::Mat labels, stats, centroids;
    const auto numLabels = connectedComponentsWithStats(dst, labels, stats, centroids);

    // define the criteria to stop and refine the corners
    const auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
    const auto winSize = cv::Size(5, 5);
    const auto zeroZone = cv::Size(-1, -1);
    std::vector<cv::Point2f> corners{};

    for (int i = 1; i < numLabels; i++) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) > 10) {  // filter out small blobs
            const auto centroidX = static_cast<float>(centroids.at<double>(i, 0));
            const auto centroidY = static_cast<float>(centroids.at<double>(i, 1));
            cv::Point2f centroid(centroidX, centroidY);
            corners.push_back(centroid);
        }
    }

    cornerSubPix(dst, corners, winSize, zeroZone, criteria);

    return corners;
}

void mouseCallback(const int event, const int x, const int y, int flags, void* userData) {
    const auto img = static_cast<cv::Mat*>(userData);

    if (event == cv::EVENT_LBUTTONDOWN) {
        // Clear the old traces
        imshow(WINDOW, *img);

        // Draw on a clone to maintain the clean image
        auto clone = img->clone();

        std::cout << "Lost at: " << x << ' ' << y << '\n';

        lostPoint = Point2D(static_cast<float>(x), static_cast<float>(y));
        vector<Point2D> out;
        const auto results = navigate(myMap, lostPoint, out);

        // The lost point
        circle(clone, cv::Point(x, y), 6, cv::Scalar(0, 0, 0), -1);

        // The visible points
        for (const auto& visible : out) {
            circle(clone, cv::Point(static_cast<int>(visible.x), static_cast<int>(visible.y)),
                6, cv::Scalar(169, 169, 169), -1
            );
        }

        // The found point
        for (const auto& result : results) {
            circle(clone, cv::Point(static_cast<int>(result.x), static_cast<int>(result.y)),
                6, cv::Scalar(255, 255, 0), -1
            );
        }

        imshow(WINDOW, clone);

        std::cout << "\n==================== NO GPS LOCALIZATION ====================== \n";
    }
}