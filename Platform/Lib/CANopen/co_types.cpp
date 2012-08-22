/*
 * co_types : constant and structure definitions for co_master lib 
 * author : Mike Hopkins
 */

#include "co_types.h"

/* defines primitive data type sizes */
const size_t co_data_size[CO_MAX_DATA_TYPE] = { 
  1, // CO_UNSIGNED8
  2, // CO_UNSIGNED16
  3, // CO_UNSIGNED24
  4, // CO_UNSIGNED32
  5, // CO_UNSIGNED40
  6, // CO_UNSIGNED48
  7, // CO_UNSIGNED56
  8, // CO_UNSIGNED64
  1, // CO_INTEGER8
  2, // CO_INTEGER16
  3, // CO_INTEGER24
  4, // CO_INTEGER32
  5, // CO_INTEGER40
  6, // CO_INTEGER48
  7, // CO_INTEGER56
  8, // CO_INTEGER64
  4, // CO_REAL32
  8, // CO_REAL64
};
