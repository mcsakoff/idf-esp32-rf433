#pragma once

#include "driver/rf_receiver.h"
#include "rf433_parser.h"

/*
 The protocol found in Shenzhen King-Serry Electronics Co. Ltd's devices.
 The devices of this manufacturer are offered under different names (e.g.: Smernit).

 It looks similar to NEC protocol used for IR, that's why it is called NEC here.
*/

/**
 * @brief Creat a new parser
 *
 * @return
 *      Handle of the parser or NULL
 */
parser_t *nec_parser_new();
