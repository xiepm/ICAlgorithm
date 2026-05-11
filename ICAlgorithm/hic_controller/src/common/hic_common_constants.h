#ifndef ecConstants_H_
#define ecConstants_H_
//------------------------------------------------------------------------------
// Copyright (c) 2001-2013 Energid Technologies. All rights reserved.
//
/// @file hic_common_constants.h
/// @brief A collection of constant values, both physics and programming constants.
//
//------------------------------------------------------------------------------
#include "hic_common_types.h"


#define		numofJoints			10
#define		spaceDOF			6

/**\namespace Ec
 * \brief Energid common namespace
*/

/// gives a representation of true
const EcBoolean   EcTrue               = true;

/// gives a representation of false
const EcBoolean   EcFalse              = false;

/// gives an accurate value for the number Pi
const EcReal      EcPi                 = 3.14159265358979324;

/// gives an accurate value for two times Pi
const EcReal      Ec2Pi                = 6.28318530717958650;

/// gives an accurate value for Pi divided by two
const EcReal      EcHalfPi             = 1.57079632679489662;

/// gives an accurate value for the number E
const EcReal      EcE                  = 2.71828182845904524;

/// gives an accurate value for the square root of 2
const EcReal      EcSqrt2              = 1.41421356237309505;

/// gives an accurate value for the square root of 3
const EcReal      EcSqrt3              = 1.73205080756887729;

/// gives an accurate value for the square root of 5
const EcReal      EcSqrt5              = 2.23606797749978969;

/// a big 64-bit floating-point number
const EcReal      EcBIG                = 1e300;

/// a small 64-bit floating-point number
const EcReal      EcSMALL              = 1e-300;


/// multiplicative conversion from inches to meters
const EcReal      EcIN2M               = 0.02540000000000000;

/// multiplicative conversion from meters to inches
const EcReal      EcM2IN               = 39.37007874015748;

/// multiplicative conversion from meters to feet
const EcReal      EcM2FT               = 3.2808398950131235;

/// multiplicative conversion from radians to degrees
const EcReal      EcRAD2DEG            = 57.29577951308232088;

/// multiplicative conversion from degrees to radians
const EcReal      EcDEG2RAD            = 0.0174532925199432958;



/// SI Powers of 1000
/// small numbers to measure numerical errors
const EcReal      EcNANO               = 1e-9;
const EcReal      EcMICRO              = 1e-6;
const EcReal      EcMILLI              = 1e-3;

const EcReal      EcKILO               = 1e3;
const EcReal      EcMEGA               = 1e6;
const EcReal      EcGIGA               = 1e9;

/// IEC Powers of 1024
const EcReal      EcKIBI               = 1 << 10;
const EcReal      EcMEBI               = 1 << 20;
const EcReal      EcGIBI               = 1 << 30;

/// a null pointer variable
#if !defined(BOOST_NO_CXX11_NULLPTR) && !defined(__VXWORKS__)
   #define EcNULL                      nullptr
#else
   #define EcNULL                      0
#endif

#endif // ecConstants_H_
