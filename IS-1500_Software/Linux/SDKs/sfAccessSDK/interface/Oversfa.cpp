#ifdef WIN32
#include "windows.h"
#endif
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include "libsfaccess.h"

//need for programm
#include <string>
#include "interface.h"
#include "Oversfa.h"

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


SfAccess sfaO;
// Member functions definitions including constructor
Oversfa::Oversfa()
:nbr(10)
{

  // sfa = a;
  std::cout << "object is created" << std::endl;
  // std::cout << "Try to open sfAcccess" << std::endl;
  // sfa.open();
  // std::cout << "sfAcccess is opened" << std::endl;
}

bool Oversfa::overInit()//Oversfa &oversfa)
{
  // SfAccess& sfa = oversfa.sfa;


  std::cout << "Try to open sfAcccess" << std::endl;
  sfaO.open();
  std::cout << "sfAcccess is opened" << std::endl;
  // sfa.getTrackingDataLatest(&trkValid, &trkData);
  // std::cout << trkData.trkPos[0] << std::endl;
  // std::cout << "sfAcccess is opened" << std::endl;
  return true;
}

float Oversfa::overGetData()
{
  SfAccess::TrkData trkData;
  bool trkValid = false;
  sfaO.getTrackingDataLatest(&trkValid, &trkData);
  // while(true)
  // {
  //
  //   std::cout << trkData.trkPos[0] << std::endl;
  // }

  return trkData.trkPos[0];
}
