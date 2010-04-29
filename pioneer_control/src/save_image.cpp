#include <vector>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
  int count = 1;
  if (argc > 1)
    count = atoi(argv[1]);

  VideoCapture cap(0);
  if (!cap.isOpened())
  {
    cerr << "Failed to open camera!" << endl;
    return -1;
  }

  for (int i = 0; i < count; i++)
  {
    stringstream filepath;
    filepath << "~/sandbox/images/" << i << ".png";
    Mat frame;
    cap >> frame;
    imwrite(filepath.str(), frame);
  }
  return 0;
}
