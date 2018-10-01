#include <iostream>
#include "interface.h"
#include <string>
#include <vector>

int main ()
{
   std::cout << "Start the program" << std::endl;
   std::vector<float> v;
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);


   while(true)
   {
     overInit();
     v = overGetData();
     // std::cout << v[0] << " " << v[1] << " " << v[2] << " "
     // << v[3] << " " << v[4] << " " << v[5] << std::endl;
     // overClose();
     const float* p1 = &v[0];
     const float* p2 = &v[1];
     const float* p3 = &v[2];
     const float* p4 = &v[3];
     const float* p5 = &v[4];
     const float* p6 = &v[5];

     std::cout << "***************start*****************" << '\n';
     std::cout << "the int at address " << p1 << " is " << *p1 << '\n';
     std::cout << "the int at address " << p2 << " is " << *p2 << '\n';
     std::cout << "the int at address " << p3 << " is " << *p3 << '\n';
     std::cout << "the int at address " << p4 << " is " << *p4 << '\n';
     std::cout << "the int at address " << p5 << " is " << *p5 << '\n';
     std::cout << "the int at address " << p6 << " is " << *p6 << '\n';
     std::cout << "****************end******************" << '\n';
   }



   std::cout << "End the programm" << std::endl;
   return 0;
}
