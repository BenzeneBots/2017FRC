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
{100, 120, 10},
{200, 150, 20},
{300, 180, 30},
{400, 200, 40},
{500, 220, 50},
{600, 240, 60},
{700, 300, 70},
{800, 340, 80},
};

results interp(int myrange); // function prototype

#endif /* Lookup_h */
