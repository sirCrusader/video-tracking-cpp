#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <unistd.h>

using namespace cv;

const double CENTER = 0.5;
const std::string LEFT = "left";
const std::string RIGHT = "right";
const std::string UP = "up";
const std::string DOWN = "down";

std::pair<std::string, std::string> computeDirection(double x, double y)
{
  std::string xDir;
  std::string yDir;

  if (x < CENTER)
  {
    xDir = LEFT;
  }
  else if (x > CENTER)
  {
    xDir = RIGHT;
  }
  if (y < CENTER)
  {
    yDir = UP;
  }
  else if (y > CENTER)
  {
    yDir = DOWN;
  }

  return std::make_pair(xDir, yDir);
}

std::pair<double, double> computeAcceleration(const std::string &xDir, const std::string &yDir, double x, double y)
{
  double xAccel = 0.0;
  double yAccel = 0.0;
  if (xDir == LEFT)
  {
    xAccel = (CENTER - x);
  }
  else if (xDir == RIGHT)
  {
    xAccel = (x - CENTER);
  }
  if (yDir == UP)
  {
    yAccel = (CENTER - y);
  }
  else if (yDir == DOWN)
  {
    yAccel = (y - CENTER);
  }

  return std::make_pair(xAccel * 100 * 2, yAccel * 100 * 2);
}


void gimbalNavigator(double x, double y, double capWidth, double capHeight)
{
  x = x / capWidth;
  y = y / capHeight;
  auto dirVal = computeDirection(x, y);
  auto accelVal = computeAcceleration(dirVal.first, dirVal.second, x, y);
  std::cout << "X-DIR: " << dirVal.first << ", Y-DIR: " << dirVal.second << ", X-ACCEL: " << accelVal.first << ", Y-ACCEL: " << accelVal.second << std::endl;
}

void drawBox(cv::Mat &img, cv::Rect bbox)
{
  cv::rectangle(img, bbox, cv::Scalar(255, 0, 255), 3, 1);
  cv::putText(img, "Tracking", cv::Point(75, 75), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}

cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();

cv::Rect2d roi;
bool isTrackingActive = true;
bool isRoiUpdated = false;
bool isTrackerInited = false;
int height = 100;
int width = 100;

void on_mouse(int event, int x, int y, int flags, void *userdata)
{
  if (event == cv::EVENT_LBUTTONUP)
  {
    if (isTrackingActive)
    {
      roi = cv::Rect2d(x, y, 100, 100); // width and height are fixed at 100 for now
      isRoiUpdated = true;
    }
  }
  else if (event == cv::EVENT_RBUTTONUP)
  {
    roi = cv::Rect2d();
    isRoiUpdated = false;
  }
}

int main()
{
  cv::VideoCapture cap(0);
  if (!cap.isOpened())
  {
    std::cerr << "Error: Could not open camera." << std::endl;
    return -1;
  }

  double capWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  double capHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

  cv::namedWindow("Tracking");
  cv::setMouseCallback("Tracking", on_mouse);

  while (cap.isOpened())
  {
    usleep(300000); // try to fix mouse and waitKey latency

    cv::Mat img;
    bool success = cap.read(img);
    if (!success)
    {
      std::cerr << "Error: Could not read frame." << std::endl;
      break;
    }

    if (isTrackingActive == true && isRoiUpdated == true && !roi.empty())
    {
      tracker->init(img, roi); // init or re-init tracker
      isTrackerInited = true;
      isRoiUpdated = false;
    }
    else if (isTrackingActive == true && isTrackerInited == true && !roi.empty())
    {
      cv::Rect bbox;
      success = tracker->update(img, bbox); // update tracker
      if (success)
      {
        gimbalNavigator(bbox.x, bbox.y, capWidth, capHeight);
      }
    }

    if (success && isTrackingActive == true && !roi.empty())
    {
      drawBox(img, roi);
    }

    double fps = cv::getTickFrequency() / (cv::getTickCount() - cv::getTickCount());
    cv::putText(img, std::to_string(static_cast<int>(fps)), cv::Point(75, 50), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);

    cv::imshow("Tracking", img);

    if (cv::waitKey(1) == 'q')
    {
      break;
    }
  }

  return 0;
}
