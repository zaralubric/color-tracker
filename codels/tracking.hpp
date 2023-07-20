/*
 * Copyright (c) 2022-2022 LAAS/CNRS
 *
 * Author: Felix Ingrand - LAAS/CNRS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>


using namespace std;

namespace Tracking
{
bool detectObject(cv::Mat &image, int b, int g, int r, int tolerance, double &x, double &y, bool debug=false) {

        // Create the mask &initialize it to white (no color detected)
        auto mask = cv::Mat(image.size(), image.type());
        // Create the thresholded image
        auto bgr = image.clone();

        // We create the mask
        cv::inRange(bgr, cv::Scalar(b - tolerance, g - tolerance, r - tolerance), cv::Scalar(b + tolerance, g + tolerance, r + tolerance), mask);

        // Kernel for the morphological operations
        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

        // Morphological opening (inverse because we have white pixels on black background)
        cv::dilate(mask, mask, kernel, cv::Size(5, 5)); // 1, 1
        cv::erode(mask, mask, kernel, cv::Size(5, 5)); // 1, 1

        // Get Image Moments
        cv::Moments m = cv::moments(mask, true);
        double m10 = m.m10;
        double m01 = m.m01;
        double mA = m.m00;

        // delete *kernel;
        // delete *mask;
        // delete bgr;

        if (mA > 1000) {
                x = m10 / mA;
                y = m01 / mA;
                cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
        }
        else {
                x = -1;
                y = -1;
                return false;
        }

        if (debug) {
                cv::imshow("Image Mask", mask);
                cv::imshow("Camera Image", image);
                cv::waitKey(1);
        }

        return true;
}

void imageToWorld(double x, double y, double &xw, double &yw, double &zw, double fx, double fy, double cx, double cy, double z) {
        xw = (x - cx) * z / fx;
        yw = (y - cy) * z / fy;
        zw = z;

        // Convert to world coordinates
}

} // namespace Tracking


#endif