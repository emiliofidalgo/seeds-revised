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

#include "Superpixel.h"

Superpixel::Superpixel() :
    id(-1)
{
    reset();
}

Superpixel::Superpixel(const int ids) :
    id(ids)
{
    reset();
}

void Superpixel::addPixel(const int x, const int y, const cv::Vec3b& pixel)
{
    total_pixels++;

    double x_val = static_cast<double>(x); this->x += x_val;
    double y_val = static_cast<double>(y); this->y += y_val;
    double L_val = static_cast<double>(pixel[0]); L += L_val; sL += SQR(L_val);
    double a_val = static_cast<double>(pixel[1]); a += a_val; sa += SQR(a_val);
    double b_val = static_cast<double>(pixel[2]); b += b_val; sb += SQR(b_val);
}

void Superpixel::reset()
{
    total_pixels = 0;
    weight = 0.0;
    x = 0.0;
    y = 0.0;
    L = 0.0; sL = 0.0;
    a = 0.0; sa = 0.0;
    b = 0.0; sb = 0.0;

    desc = cv::Mat::zeros(6, 1, CV_64F);
    sigma = cv::Mat::eye(6, 6, CV_64F);
}

void Superpixel::computeDescription(const cv::Mat& image)
{
    assert(total_pixels > 0);

    double npixels = static_cast<double>(total_pixels);

    // Computing means
    weight = npixels / static_cast<double>(image.rows * image.cols); desc(0, 0) = weight;
    x /= npixels; x /= image.cols; desc(1, 0) = x;
    y /= npixels; y /= image.rows; desc(2, 0) = y;
    L /= npixels; desc(3, 0) = L;
    a /= npixels; desc(4, 0) = a;
    b /= npixels; desc(5, 0) = b;

    // Computing standard deviations
    sL /= npixels;
    sa /= npixels;
    sb /= npixels;
    sL -= SQR(L);
    sa -= SQR(a);
    sb -= SQR(b);
    sL = std::sqrt(sL); sigma(3, 3) = 1.0 / (1.0 + sL * sL);
    sa = std::sqrt(sa); sigma(4, 4) = 1.0 / (1.0 + sa * sa);
    sb = std::sqrt(sb); sigma(5, 5) = 1.0 / (1.0 + sb * sb);
}

std::string Superpixel::toString()
{
    std::stringstream ss;

    ss << "---" << std::endl;
    ss << "Superpixel: " << id << std::endl;
    ss << "w: " << weight << std::endl;
    ss << "x: " << x << std::endl;
    ss << "y: " << y << std::endl;
    ss << "L: " << L << ", sL: " << sL << std::endl;
    ss << "a: " << a << ", sa: " << sa << std::endl;
    ss << "b: " << b << ", sb: " << sb << std::endl;

    return ss.str();
}

double Superpixel::distance(const Superpixel& a, const Superpixel& b, int dist_type, cv::Mat weights)
{
    if (dist_type == SPX_DIST_L2)
    {
        return distance_L2(a, b);
    }
    else if (dist_type == SPX_DIST_L1)
    {
        return distance_L1(a, b);
    }
    else if (dist_type == SPX_DIST_CUSTOM)
    {
        return distance_CUSTOM(a, b, weights);
    }
    else
    {
        return DBL_MAX;
    }
}

double Superpixel::distance_L2(const Superpixel& a, const Superpixel& b)
{    
    return cv::norm(a.desc, b.desc, cv::NORM_L2);
}

double Superpixel::distance_L1(const Superpixel& a, const Superpixel& b)
{
    return cv::norm(a.desc, b.desc, cv::NORM_L1);
}

double Superpixel::distance_CUSTOM(const Superpixel& a, const Superpixel& b, cv::Mat weights)
{
    cv::Mat w = cv::Mat::eye(6, 6, CV_64F);
    if (!weights.empty())
    {
        weights.copyTo(w);
    }

    cv::Mat diff = a.desc - b.desc;
    cv::Mat t = diff.t() * w * a.sigma * diff;

    return sqrt(t.at<double>(0,0));
}
