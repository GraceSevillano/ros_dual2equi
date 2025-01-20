#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

#define VERBOSE
//#define MEASUREMAPPINGTIME

// Global variables
image_transport::Publisher pub_equirectangular_image;
sensor_msgs::ImagePtr EquiImage_msg;
cv::Mat EquiImage;
cv_bridge::CvImagePtr pt_DualImage;

int imWidth, imHeight;
unsigned int nbPixels;
int *coordMapping[4] = {nullptr, nullptr, nullptr, nullptr}; // equirectangular and dual fisheye image pixel coordinates

// Function prototypes
void init(const std::string &coordinatesTableFilename);
void imageCallback(const sensor_msgs::ImageConstPtr& Image);

int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "dual2equi_fromtable");
  ros::NodeHandle nH;

#ifdef VERBOSE
  ROS_INFO("dual2equi_fromtable::main");
#endif

  // ROS read launch file parameters
  ros::NodeHandle nHp("~"); // to get the parameters for that node
  std::string inputImagesTopic, outputImagesTopic, coordinatesTableFilename;
  nHp.param("inputImagesTopic", inputImagesTopic, std::string(""));
  nHp.param("outputImagesTopic", outputImagesTopic, std::string(""));
  nHp.param("coordinatesTableFilename", coordinatesTableFilename, std::string(""));

  nHp.param("imWidth", imWidth, 0);
  nHp.param("imHeight", imHeight, 0);

  nbPixels = imWidth * imHeight;

  init(coordinatesTableFilename);

  // ROS Listener
  image_transport::ImageTransport i_t(nH);
  image_transport::Subscriber sub_image = i_t.subscribe(inputImagesTopic, 10000, imageCallback);

  // ROS Publisher
  pub_equirectangular_image = i_t.advertise(outputImagesTopic, 1);

  ros::spin();

  // After killing ROS node
  for (unsigned int c = 0; c < 4; c++)
    if (coordMapping[c] != nullptr)
    {
      delete[] coordMapping[c];
      coordMapping[c] = nullptr;
    }

  return 0;
}

void init(const std::string &coordinatesTableFilename)
{
  if (nbPixels == 0)
  {
    ROS_INFO("dual2equi_fromtable::init: nbPixels == 0");
    return;
  }

  // Get pixel coordinates mapping
  for (unsigned int c = 0; c < 4; c++)
    coordMapping[c] = new int[nbPixels];

  std::ifstream ficCoords(coordinatesTableFilename.c_str());

  if (!ficCoords) {
    ROS_ERROR("Failed to open coordinates table file: %s", coordinatesTableFilename.c_str());
    return;
  }

  unsigned int pix = 0;
  while (!ficCoords.eof() && pix < nbPixels)
  {
    ficCoords >> coordMapping[0][pix] >> coordMapping[1][pix] >> coordMapping[2][pix] >> coordMapping[3][pix];
    pix++;
  }

  if (pix != nbPixels) {
    ROS_ERROR("Coordinates table does not have expected number of pixels. Read %d pixels, expected %d", pix, nbPixels);
  }

  // Initialize the image to be published
  EquiImage.create(imHeight, imWidth, CV_8UC3);
}

void imageCallback(const sensor_msgs::ImageConstPtr& DualImage_msg)
{
#ifdef VERBOSE
  ROS_INFO("Image received: width=%d, height=%d", DualImage_msg->width, DualImage_msg->height);
#endif

  pt_DualImage = cv_bridge::toCvCopy(DualImage_msg, sensor_msgs::image_encodings::BGR8);

  if (pt_DualImage->image.empty()) {
    ROS_ERROR("Empty image received");
    return;
  }

#ifdef MEASUREMAPPINGTIME
  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();
#endif

  int *pt_ue = coordMapping[0], *pt_ve = coordMapping[1], *pt_ud = coordMapping[2], *pt_vd = coordMapping[3];

  // Optimized version with specific knowledge on the equirect coordinates
  unsigned char *pt_equi = EquiImage.data;

  for (unsigned int p = 0; p < nbPixels; p++, pt_ue++, pt_ve++, pt_ud++, pt_vd++)
  {
    if (*pt_ud >= 0 && *pt_ud < DualImage_msg->width && *pt_vd >= 0 && *pt_vd < DualImage_msg->height) {
      memcpy(pt_equi + 3 * ((*pt_ve) * imWidth + (*pt_ue)), pt_DualImage->image.data + 3 * ((*pt_vd) * DualImage_msg->width + (*pt_ud)), 3);
    } else {
      ROS_WARN("Out of bounds coordinate: pt_ud=%d, pt_vd=%d", *pt_ud, *pt_vd);
    }
  }

#ifdef MEASUREMAPPINGTIME
  end_ = ros::WallTime::now();

  double execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO_STREAM("Execution time (ms): " << execution_time);
#endif

  EquiImage_msg = cv_bridge::CvImage(DualImage_msg->header, "bgr8", EquiImage).toImageMsg();
  pub_equirectangular_image.publish(EquiImage_msg);
  ROS_INFO("Image processed and published");
}
