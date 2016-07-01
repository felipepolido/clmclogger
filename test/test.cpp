#include "mrdplot/mrdplot.h"
#include "mrdplot/MRDLogger.h"
#include <math.h>
#include <iostream>

void aa()
{
  MRDPlot mrd;
  mrd.alloc(3, 1000);
  mrd._freq = 500;
  mrd.setChannelNameAndUnit("a", "-", 0);
  mrd.setChannelNameAndUnit("b", "-", 1);
  mrd.setChannelNameAndUnit("c", "-", 2);

  float ctr = 0.3;
  for (int i = 0; i < 1000; i++) {
    for (int j = 0; j < 3; j++) {
      mrd._data[i*3+j] = ctr + 1.;
      ctr++;
    }
  }

  mrd.writeToFile("1asf.mrd");

  MRDPlot M;
  M.readFromFile("1asf.mrd");
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      printf("%g\n", M._data[i*3+j]);
    }
  }
}


void newstuff()
{
  MRDLogger log(3,true);
  int a = 2;
  double b = 2.3;
  long c = -5;
  bool d = true; 

  log.addChannel("a", "-", &a);
  log.addChannel("b", "-", &b);
  log.addChannel("c", "-", &c);
  log.addChannel("d", "-", &d);

  printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  log.saveData();
  
  a++; b++; c++; d++;
  printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  log.saveData();
  
  a++; b++; c++; d++;
  printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  log.saveData();
  
  a++; b++; c++; d++;
  printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  log.saveData();

  a++; b++; c++; d++;
  printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  log.saveData();

  a++; b++; c++; d++;
  printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  log.saveData();

  a++; b++; c++; d++;
  printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  log.saveData();
  
  log.writeToFile("asdf");

  // float e = 0;
  // log.addChannel("e", "-", &e);

  // std::cout << "Read file" << std::endl;
  // log.readFromFile("asdf.mrd");

  // a = b = c = d = 0;
  // while(log.hasMoreData()) {
  //   log.popData();
  //   printf("a %d, b %g, c %ld, d %d\n", a,b,c,d);
  // } 
  std::cout << "Finished example" << std::endl;
}

int main()
{
  newstuff();
}
