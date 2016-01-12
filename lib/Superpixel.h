/**
* The code is published under the BSD 3-Clause:
*
* Copyright (c) 2016, Emilio Garcia-Fidalgo (emilio.garcia@uib.es)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef SUPERPIXEL_H
#define SUPERPIXEL_H

#include <opencv2/opencv.hpp>

#define SQR(x) ((x)*(x))

#define SPX_DIST_L2 0
#define SPX_DIST_L1 1
#define SPX_DIST_CUSTOM 2

class Superpixel
{
public:
    Superpixel();
    Superpixel(const int ids);

    void addPixel(const int x, const int y, const cv::Vec3b& pixel);
    void reset();
    void computeDescription(const cv::Mat& image);
    static double distance(const Superpixel& a, const Superpixel& b, int dist_type = SPX_DIST_CUSTOM, cv::Mat weights = cv::Mat());
    std::string toString();

    unsigned total_pixels;
    int id;
    double weight;
    double x;
    double y;
    double L, sL;
    double a, sa;
    double b, sb;    

    // cv::Mat for fast operations
    cv::Mat_<double> desc;
    cv::Mat_<double> sigma;

private:
    static double distance_L2(const Superpixel& a, const Superpixel& b);
    static double distance_L1(const Superpixel& a, const Superpixel& b);
    static double distance_CUSTOM(const Superpixel& a, const Superpixel& b, cv::Mat weights);
};

#endif // SUPERPIXEL_H
