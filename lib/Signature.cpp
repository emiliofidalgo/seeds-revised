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
#include "Signature.h"

Signature::Signature()
{
    reset();
}

Signature::Signature(const int nspixels)
{
    reset();
    resize(nspixels);
}

void Signature::resize(const int nspixels)
{
    for (int i = 0; i < nspixels; i++)
    {
        Superpixel spx(i);
        spixels.push_back(spx);
    }
}

void Signature::reset()
{
    spixels.clear();
}

std::string Signature::toString()
{
    std::stringstream ss;

    for (int i = 0; i < spixels.size(); i++)
    {
        ss << spixels[i].toString();
    }

    return ss.str();
}

double Signature::distance(const Signature& a, const Signature& b, int redop_left, int redop_right, int spx_dist_type)
{
    cv::Mat_<double> dists(a.spixels.size(), b.spixels.size(), 0.0);

    // Computing distances between all pairs of superpixels
    #pragma omp parallel for
    for (int i = 0; i < a.spixels.size(); i++)
    {
        for (int j = 0; j < b.spixels.size(); j++)
        {
            double dist = Superpixel::distance(a.spixels[i], b.spixels[j], spx_dist_type);
            dists(i, j) = dist;
        }
    }

    // --- Reducing the distance matrix to a final expression
    // First reduction
    cv::Mat aux_right;
    int rtype_right;
    switch (redop_right)
    {
        case SGN_REDOP_MAX:
            rtype_right = CV_REDUCE_MAX;
            break;
        case SGN_REDOP_MIN:
            rtype_right = CV_REDUCE_MIN;
            break;
        case SGN_REDOP_AVG:
            rtype_right = CV_REDUCE_AVG;
            break;
        default:
            rtype_right = CV_REDUCE_MIN;
            break;
    }
    cv::reduce(dists, aux_right, 1, rtype_right);

    // Second reduction
    cv::Mat aux_left;
    int rtype_left;
    switch (redop_left)
    {
        case SGN_REDOP_MAX:
            rtype_left = CV_REDUCE_MAX;
            break;
        case SGN_REDOP_MIN:
            rtype_left = CV_REDUCE_MIN;
            break;
        case SGN_REDOP_AVG:
            rtype_left = CV_REDUCE_AVG;
            break;
        default:
            rtype_left = CV_REDUCE_MIN;
            break;
    }
    cv::reduce(aux_right, aux_left, 0, rtype_left);

    return aux_left.at<double>(0, 0);
}
