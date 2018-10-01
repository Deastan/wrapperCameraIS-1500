#include "interface.h"

using namespace std;

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

// TODO Use pointer of reference to this object
// Define as a global value
// Not the best !
bool alreadyOpen = false;
SfAccess sfa;

// Open the "Buffer of the camera"
bool overInit()//Oversfa &oversfa)
{
  if(alreadyOpen == false)
  {
    std::cout << "Try to open sfAcccess" << std::endl;
    sfa.open();
    std::cout << "sfAcccess is opened" << std::endl;
    alreadyOpen = true;
  }

  return true;
}

// Get the data of the IMU which transforms and saves it inside a vector of float
// Vector v is organised as roll  pitch    yaw   posx   posy   posz
// OUTPUT std::vector<float> v
// TODO Improvement :
// - Replace the output with an object if we want to add a collected picture
vector<float> overGetData()
{
  SfAccess::TrkData trkData;
  bool trkValid = false;
  sfa.getTrackingDataLatest(&trkValid, &trkData);

  // v is a table of float with the data of the IMU of the camera
  // organised as roll  pitch    yaw   posx   posy   posz
  vector<float> v;
  v.push_back(RAD2DEG(trkData.trkRot[0]));
  v.push_back(RAD2DEG(trkData.trkRot[1]));
  v.push_back(RAD2DEG(trkData.trkRot[2]));
  v.push_back(trkData.trkPos[0]);
  v.push_back(trkData.trkPos[1]);
  v.push_back(trkData.trkPos[2]);

  // return the vector containing the imu's data.
  return v;
}

// Close the "Buffer of the camera"
bool overClose()
{
  sfa.close();
  return true;
}
