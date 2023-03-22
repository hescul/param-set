#include "NoGPS.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <limits>
#include "Timer.h"

constexpr auto EYE_RANGE = 100;

using namespace std;

vector<Point2D> navigate(const vector<Point2D>& mesh, const Point2D& lostPoint, vector<Point2D>& out) {
	// //////// offline phase ////////
    std::cout << "Map Size: " << mesh.size() << endl;

    Timer t;
    const auto myMap = offlinePhase(mesh);
    std::cout << "[t] Offline phase time taken: " << t.elapsed() << " seconds\n";

    //////// online phase ////////
    Point2D closestPoint;
    float minDistance = std::numeric_limits<float>::max();
    
    for (const auto& x : mesh) {
        if (minDistance > sqrt(pow(lostPoint.x - x.x, 2) + pow(lostPoint.y - x.y, 2))) {
            minDistance = sqrt(pow(lostPoint.x - x.x, 2) + pow(lostPoint.y - x.y, 2));
            closestPoint = x;
        }
    }
    std::cout << "Closest point: " << closestPoint.x << " " << closestPoint.y << endl;

    vector<Point2D> seeablePoints;
    for (auto x : mesh)
    {
        if (sqrt(pow(lostPoint.x - x.x, 2) + pow(lostPoint.y - x.y, 2)) < EYE_RANGE)
        {
            seeablePoints.push_back(x);
        }
    }
    out = seeablePoints;

	const auto seeablePs = PS_Generator(closestPoint, seeablePoints);
    std::cout << "Number of seeable points: " << seeablePoints.size() - 1 << endl;

    t.reset();
    const auto resultPoints = onlinePhase(myMap, seeablePs, mesh);
    std::cout << "[t] Online phase time taken: " << t.elapsed() << " seconds\n";

    std::cout << "----------------------------------------------" << endl;
    std::cout << "Result:" << endl;
    for (const auto& result : resultPoints) {
        std::cout << result.x << " " << result.y << endl;
    }
    return resultPoints;
}

vector<Point2D> noGPSNavigation() {
    float offline_t[3];
    float online_t[3];
    vector<Point2D> resultPoints;
    for (int t_index = 0; t_index < 3; t_index++)
    {
        // //////// offline phase ////////
        vector<Point2D> mesh;
        string buffer;
        ifstream HCMUTCampus("HCMUT_campus_test2.txt");

        // Use a while loop together with the getline() function to read the file line by line
        while (getline(HCMUTCampus, buffer))
        {
            // Read the input from the file
            string x;
            string y;
            bool xStart = false;
            bool yStart = false;
            for (char c : buffer)
            {
                if (c == '(')
                {
                    xStart = true;
                    continue;
                }
                if (c == ',')
                {
                    xStart = false;
                    yStart = true;
                    continue;
                }
                if (xStart)
                {
                    x += c;
                }
                if (yStart)
                {
                    y += c;
                }
            }
            Point2D p(stof(x), stof(y));
            mesh.push_back(p);
        }
        std::cout << "Map Size: " << mesh.size() << endl;
        vector<PS> myMap;

        Timer t;
        myMap = offlinePhase(mesh);
        offline_t[t_index] = t.elapsed();
        std::cout << "[t] Offline phase time taken: " << t.elapsed() << " seconds\n";

        //////// online phase ////////
        Point2D lostPoint(998 / 2, 586 / 2);
        Point2D closestPoint;
        float minDistance = std::numeric_limits<float>::max();
        ;
        for (auto x : mesh)
        {
            if (minDistance > sqrt(pow(lostPoint.x - x.x, 2) + pow(lostPoint.y - x.y, 2)))
            {
                minDistance = sqrt(pow(lostPoint.x - x.x, 2) + pow(lostPoint.y - x.y, 2));
                closestPoint = x;
            }
        }
        std::cout << "Closest point: " << closestPoint.x << " " << closestPoint.y << endl;

        vector<Point2D> seeable_points;
        for (auto x : mesh)
        {
            if (sqrt(pow(lostPoint.x - x.x, 2) + pow(lostPoint.y - x.y, 2)) < EYE_RANGE)
            {
                seeable_points.push_back(x);
            }
        }

        PS seeable_PS;
        seeable_PS = PS_Generator(closestPoint, seeable_points);
        std::cout << "Number of seeable points: " << seeable_points.size() - 1 << endl;

        t.reset();
        resultPoints = onlinePhase(myMap, seeable_PS, mesh);
        online_t[t_index] = t.elapsed();
        std::cout << "[t] Online phase time taken: " << t.elapsed() << " seconds\n";

        std::cout << "----------------------------------------------" << endl;
        std::cout << "Result:" << endl;
        for (auto result : resultPoints)
        {
            std::cout << result.x << " " << result.y << endl;
        }
    }
    cout << "+++++++++ Offline time: " << (offline_t[0] + offline_t[1] + offline_t[2]) / 3 << endl;
    cout << "+++++++++ Online time: " << (online_t[0] + online_t[1] + online_t[2]) / 3 << endl;
    return resultPoints;
}