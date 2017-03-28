//
//  Lookup.cpp

//

#include "Lookup.h"

// function to linear interpolate to find RPM and angle from distance look up table

results Lookup::interp(int myrange)
{
    // setup variables
    
    int i, j, k;
    
    results final;
    final.speed = 0;
    final.position = 0;
    
    for( i = 0; i < size-1; i++ )  // loop through table
    {
      if ( lut[i].distance <= myrange && lut[i+1].distance >= myrange ) // find out if in range of table
      {
          double diffrange = myrange - lut[i].distance;  // compare to lower end of range
          double diffn = lut[i+1].distance - lut[i].distance; // calculate the delta of range
          
          // linear interpolate
          j = lut[i].speed + (lut[i+1].speed - lut[i].speed) * diffrange / diffn;
          k = lut[i].position + (lut[i+1].position - lut[i].position) * diffrange / diffn;
          
          // fill results and return 
          final.speed = j;
          final.position = k;
          
          return final;
      }
    }
    return final;  // not in range of table
}
                   
