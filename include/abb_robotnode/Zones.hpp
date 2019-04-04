#ifndef _ABB_ROBOTNODE_ZONES_HPP_
#define _ABB_ROBOTNODE_ZONES_HPP_

typedef enum
{
  ZONE_FINE = 0,
  ZONE_1,
  ZONE_2,
  ZONE_3,
  ZONE_4,
  ZONE_5,
  NUM_ZONES
} ZONE_TYPE;

typedef struct
{
  double p_tcp; // TCP path zone (mm)
  double p_ori; // Zone size for orientation (mm)
  double ori;   // Tool orientation (degrees)
} zoneVals;

static const zoneVals zoneData[NUM_ZONES] =
{
  // p_tcp (mm), p_ori (mm), ori (deg)
  {0.0,   0.0,  0.0},   // ZONE_FINE
  {0.3,   0.3,  0.03},  // ZONE_1
  {1.0,   1.0,  0.1},   // ZONE_2
  {5.0,   8.0,  0.8},   // ZONE_3
  {10.0,  15.0, 1.5},   // ZONE_4
  {20.0,  30.0, 3.0}    // ZONE_5
};

#endif
