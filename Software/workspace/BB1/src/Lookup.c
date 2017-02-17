//
//  lookup.c
//

#include <stdio.h>   // library for printing

#define size 8       // look up table size

typedef struct {int rpm; int range;} coords; // look up table data structure

coords lut[size] =   // define look up table
{
//   RPM   RANGE (in centimeters)
    {100,  500},
    {200,  600},
    {300,  700},
    {400,  800},
    {500,  900},
    {600,  1000},
    {700,  4000},
    {800,  5000}
};

// function to linear interpolate to find RPM from a estimated range look up table

int interp(coords* c, int myrange)
{
    int i;
    
    for( i = 0; i < size-1; i++ )  // loop through table
    {
      if ( c[i].range <= myrange && c[i+1].range >= myrange ) // find out if in range of table
      {
          double diffrange = myrange - c[i].range;  // compare to lower end of range
          double diffn = c[i+1].range - c[i].range; // calculate the delta of range
            
          return c[i].rpm + ( c[i+1].rpm - c[i].rpm ) * diffrange / diffn; // math for linear interpolation
      }
    }
    return 0;  // not in range of table
}

// Usage example
int mymain() {
    
    int myrpm;
    
    myrpm = interp(lut, 875); // send value of range and get RPM back
    
    printf("%d is the RPM value", myrpm); // show RPM value
    
    return 0;
}
                   
