#include "PS.h"
#include <cmath>
#include <algorithm>

using namespace std;

vector<PS> offlinePhase(const vector<Point2D>& mesh)
{
    vector<PS> result;
    for (int i = 0; i < mesh.size(); i++)
    {
        result.push_back(PS_Generator(mesh[i], mesh));
    }
    return result;
}

PS PS_Generator(const Point2D& source, const vector<Point2D>& mesh)
{
    static const double TWOPI = 6.2831853071795865;
    PS PS; // PS = Parameter Set

    ///////////////////////////// calculate help list /////////////////////////////
    vector<PS_help_node> PS_help_list;
    for (int i = 0; i < mesh.size(); i++)
    {
        // We do not need to calculate for the source point
        if (mesh[i].x == source.x && mesh[i].y == source.y)
            continue;

        PS_help_node help_node;
        help_node.point = mesh[i];
        help_node.distance = sqrt(pow(source.x - mesh[i].x, 2) + pow(source.y - mesh[i].y, 2));
        double theta = atan2(mesh[i].y - source.y, mesh[i].x - source.x);
        // double theta = atan2(mesh[i].x - source.x, mesh[i].y - source.y);
        if (theta < 0.0)
        {
            theta += TWOPI;
        }
        help_node.theta = theta;

        // Insertion Sort idea
        bool haveInserted = false;
        for (int j = 0; j < PS_help_list.size(); j++)
        {
            if (help_node.theta == PS_help_list[j].theta)
            {
                if (help_node.distance < PS_help_list[j].distance)
                {
                    PS_help_list.insert(PS_help_list.begin() + j, help_node);
                    haveInserted = true;
                    break;
                }
            }

            if (help_node.theta < PS_help_list[j].theta)
            {
                PS_help_list.insert(PS_help_list.begin() + j, help_node);
                haveInserted = true;
                break;
            }
        }

        if (!haveInserted)
            PS_help_list.push_back(help_node);
    }

    ///////////////////////////// calculate PS /////////////////////////////
    for (int i = 0; i < PS_help_list.size(); i++)
    {
        PS.distances.push_back(PS_help_list[i].distance);
        if (i == PS_help_list.size() - 1)
        {
            PS.angles.push_back(TWOPI - (PS_help_list[i].theta - PS_help_list[0].theta));
        }
        else
        {
            PS.angles.push_back(PS_help_list[i + 1].theta - PS_help_list[i].theta);
        }
    }
    return PS;
};

bool approx(float x, float y)
{
    return abs(x - y) <= ANGLE_ERROR;
}

vector<Point2D> onlinePhase(const vector<PS>& myMap, const PS& seeable_PS, const vector<Point2D>& mesh)
{
    vector<Point2D> resultPoints;
    for (int i = 0; i < myMap.size(); i++)
    {
        // check if seeable_PS.distances is a subset of myMap[i].distances
        bool isFind = isSubset(seeable_PS.distances, myMap[i].distances);

        if (isFind) // seeable_PS is a subset of myMap[i]
        {
            float alpha = seeable_PS.angles[0];
            int left_pointer = 0;
            int right_pointer = 0;
            float window_sum = 0;
            do
            {
                right_pointer %= myMap[i].angles.size();
                window_sum += myMap[i].angles[right_pointer];
                if (approx(window_sum, alpha))
                {
                    // cout << "Found a match in angle" << endl;
                    bool isMatch = true;
                    float accumulatedSum = 0;

                    int l = 1;
                    for (int k = (right_pointer + 1) % myMap[i].angles.size();
                        k != left_pointer;
                        k = (k + 1) % myMap[i].angles.size())
                    {
                        accumulatedSum += myMap[i].angles[k];
                        if (approx(accumulatedSum, seeable_PS.angles[l]))
                        {
                            accumulatedSum = 0;
                            l++;
                        }
                        else if (accumulatedSum > seeable_PS.angles[l])
                        {
                            isMatch = false;
                            break;
                        }
                    }
                    if (accumulatedSum != 0)
                        isMatch = false;
                    if (isMatch)
                    {
                        resultPoints.push_back(mesh[i]);
                        break;
                    };
                    window_sum -= myMap[i].angles[left_pointer];
                    // window_sum -= myMap[i].angle[right_pointer];
                    right_pointer++;
                    left_pointer++;
                }
                else if (window_sum > alpha)
                {
                    window_sum -= myMap[i].angles[left_pointer];
                    window_sum -= myMap[i].angles[right_pointer];
                    left_pointer++;
                }
                else if (window_sum < alpha)
                    right_pointer++;
            } while (left_pointer != myMap[i].angles.size());
        }
    }
    return resultPoints;
}

bool isSubset(vector<float> a, vector<float> b)
{
    sort(a.begin(), a.end());
    sort(b.begin(), b.end());
    int index = 0;
    for (auto Bnode : b)
    {
        if (a[index] == Bnode)
            index++;
    }
    return (index == a.size());
}