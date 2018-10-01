#ifdef WIN32
#include "windows.h"
#endif
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include "libsfaccess.h"
#include "interface.h"
#include <string>

using namespace std;

// Return status;
#define ERR 0
#define OK 1

// Sleep specified number of ms
#ifdef WIN32
#define SNOOZE_MS(ms) Sleep(ms)
#else // Linux
#define SNOOZE_MS(ms) usleep((ms) * 1000)
#endif

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

// Stop program?
static bool stop = false;

// Signal handler for ctrl-c
void sigHandler(int sig)
{
    cout << endl << "Stop signal received" << endl;
    stop = true;
}
// TODO Use pointer of reference to this object
// Define as a global value
// Not the best !
SfAccess sfa;

/******************************************
*
*******************************************/
bool overInit()//Oversfa &oversfa)
{
  // SfAccess& sfa = oversfa.sfa;


  std::cout << "Try to open sfAcccess" << std::endl;
  sfa.open();
  std::cout << "sfAcccess is opened" << std::endl;
  // sfa.getTrackingDataLatest(&trkValid, &trkData);
  // std::cout << trkData.trkPos[0] << std::endl;
  // std::cout << "sfAcccess is opened" << std::endl;
  return true;
}

float overGetData()
{
  SfAccess::TrkData trkData;
  bool trkValid = false;
  sfa.getTrackingDataLatest(&trkValid, &trkData);
  // while(true)
  // {
  //
  //   std::cout << trkData.trkPos[0] << std::endl;
  // }

  return trkData.trkPos[0];
}

/******************************************
*
*******************************************/








































string testFuncion(bool alpha)
{
  if (alpha == true)
  {
    return "Halo Suzi !";
  }else
  {
    return "Wir sind nicht fröhlich";
  }
}

float allInOne(bool alpha)
{
  SfAccess sfa;
  sfa.open();
  std::cout << "sfa is open !" << std::endl;
  SfAccess::TrkData trkData;
  bool trkValid = false;
  std::cout << "sfa is tacking data !" << std::endl;
  while(true)
  {
    sfa.getTrackingDataLatest(&trkValid, &trkData);
    std::cout << trkData.trkPos[0] << std::endl;
  }



  sfa.close();
  std::cout << "sfa is is close !" << std::endl;
  return trkData.trkPos[0];
}
