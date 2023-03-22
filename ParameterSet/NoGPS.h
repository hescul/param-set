#pragma once

#include <vector>

#include "PS.h"

using namespace std;

vector<Point2D> navigate(const vector<Point2D>& mesh, const Point2D& lostPoint, vector<Point2D>& out);
vector<Point2D> noGPSNavigation();