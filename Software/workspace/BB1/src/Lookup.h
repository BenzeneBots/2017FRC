//
//  Lookup.h
//

#ifndef Lookup_h
#define Lookup_h

typedef struct {int distance; int speed; int position;} params;  // look up table data structure
typedef struct {int speed; int position;} results;  // return value data structure

#define size 8       // look up table size - CHANGE HERE

static const params lut[size] = // look up table hard coded values - CHANGE HERE
{
{1, 120, 10},
{2, 150, 20},
{3, 180, 30},
{4, 200, 40},
{5, 220, 50},
{6, 240, 60},
{7, 300, 70},
{8, 340, 80},
};

namespace Lookup{
results interp(int myrange); // function prototype
}
#endif /* Lookup_h */
