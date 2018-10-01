#include <iostream>
#include "interface.h"
// #include "Oversfa.h"
#include <string>

using namespace std;

int main ()
{
   std::cout << "Start the programm" << std::endl;

   overInit();
   while(true)
   {

     std::cout << overGetData() << std::endl;
   }
   // std::cout << testFuncion(true) << std::endl;
   // // while(true)
   // // {
   // //   std::cout << "The position in is : " << allInOne(true) << std::endl;
   // // }
   // // allInOne(true);
   // Oversfa albert;
   // albert.overInit();
   //
   // while(true)
   // {
   //   std::cout << albert.overGetData() << std::endl;
   // }

   std::cout << "End the programm" << std::endl;
   return 0;
}
