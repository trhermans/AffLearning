/*
  Copyright (C) 2010 Tucker Hermans

  This library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser Public License for more details.

  You should have received a copy of the GNU General Public License
  and the GNU Lesser Public License along with Man.  If not, see
  <http://www.gnu.org/licenses/>.
*/

#include "BgSubtract.h"
#include <opencv/highgui.h>

using namespace cv;

int main()
{
    Mat bg_img = imread("testBg.png");
    Mat fg_img = imread("testFg.png");
    namedWindow("Background");
    imshow("Background", bg_img);

    namedWindow("Foreground");
    imshow("Foreground", fg_img);
    waitKey();

    BgSubtract bg(bg_img);
    Mat img_diff = bg.getContours(fg_img);

    namedWindow("Diff");
    imshow("Diff", img_diff);

    waitKey();
}
