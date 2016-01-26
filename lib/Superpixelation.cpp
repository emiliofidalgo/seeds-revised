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

#include "Superpixelation.h"

Superpixelation::Superpixelation(const cv::Mat& image, const cv::Rect& roi, const int nspixels) :
    seeds(0)
{
    image.copyTo(this->image);
    this->roi = roi;
    cv::Mat patch = this->image(this->roi);
    seeds = new SEEDSRevisedMeanPixels(patch, nspixels, 5, 1, 0.1f, 0.25f);
}

Superpixelation::~Superpixelation()
{
    delete seeds;
}

void Superpixelation::createSuperpixelation()
{
    seeds->initialize();
    seeds->iterate(2);
}

bool Superpixelation::computeSignature(const cv::Rect& roi, Signature& sign)
{
    // Check if the current ROI is inside the superpixelation
    bool is_inside = (roi & this->roi) == roi;

    if (!is_inside)
    {
        return false;
    }
    else
    {
        // Computing the new ROI according to the superpixelation
        cv::Rect new_roi = roi;
        new_roi.x -= this->roi.x;
        new_roi.y -= this->roi.y;
        seeds->computeSignature(new_roi, sign);
        return true;
    }
}

void Superpixelation::show()
{
    cv::Mat out_img;
    image.copyTo(out_img);
    cv::Mat patch = out_img(this->roi);

    // Getting contour image
    int bgr[] = {0, 0, 204};
    cv::Mat contourImage = Draw::contourImage(seeds->getLabels(), patch, bgr);
    contourImage.copyTo(patch);

    cv::imshow("Superpixelation", out_img);
    cv::waitKey(0);
}
