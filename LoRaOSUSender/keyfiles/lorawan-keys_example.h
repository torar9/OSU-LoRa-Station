/*******************************************************************************
 *
 *  File:          lorawan-keys_example.h
 * 
 *  Function:      Example for lorawan-keys.h required by LMIC-node.
 *
 *  Copyright:     Copyright (c) 2021 Leonel Lopes Parente
 *
 *  Important      ██ DO NOT EDIT THIS EXAMPLE FILE (see instructions below) ██
 * 
 *  Decription:    lorawan-keys.h defines LoRaWAN keys needed by the LMIC library.
 *                 It can contain keys for both OTAA and for ABP activation.
 *                 Only the keys for the used activation type need to be specified.
 * 
 *                 It is essential that each key is specified in the correct format.
 *                 lsb: least-significant-byte first, msb: most-significant-byte first.
 * 
 *                 For security reasons all files in the keyfiles folder (except file
 *                 lorawan-keys_example.h) are excluded from the Git(Hub) repository.
 *                 Also excluded are all files matching the pattern *lorawan-keys.h.
 *                 This way they cannot be accidentally committed to a public repository.
 * 
 *  Instructions:  1. Copy this file lorawan-keys_example.h to file lorawan-keys.h
 *                    in the same folder (keyfiles).
 *                 2. Place/edit required LoRaWAN keys in the new lorawan-keys.h file.
 *
 ******************************************************************************/

#pragma once

#ifndef LORAWAN_KEYS_H_
#define LORAWAN_KEYS_H_

// Optional: If DEVICEID is defined it will be used instead of the default defined in the BSF.
// #define DEVICEID "<deviceid>"

// Keys required for OTAA activation:

// End-device Identifier (u1_t[8]) in lsb format
#define OTAA_DEVEUI 0x46, 0x67, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70

// Application Identifier (u1_t[8]) in lsb format
#define OTAA_APPEUI 0x19, 0x4A, 0x06, 0x33, 0xDF, 0xC2, 0x2A, 0x82

// Application Key (u1_t[16]) in msb format
#define OTAA_APPKEY 0x34, 0xA5, 0x8A, 0x5C, 0xA1, 0xEA, 0xFE, 0xFD, 0x1C, 0xEF, 0x68, 0x9E, 0x44, 0xC3, 0x4C, 0x98


// -----------------------------------------------------------------------------

// Optional: If ABP_DEVICEID is defined it will be used for ABP instead of the default defined in the BSF.
// #define ABP_DEVICEID "<deviceid>"
#define ABP_DEVICEID "<deviceid>"

// Keys required for ABP activation:

// End-device Address (u4_t) in uint32_t format. 
// Note: The value must start with 0x (current version of TTN Console does not provide this).
#define ABP_DEVADDR 0x260B38E0

// Network Session Key (u1_t[16]) in msb format
#define ABP_NWKSKEY 0xEE, 0x03, 0x70, 0x4E, 0xE9, 0x47, 0xB8, 0x58, 0x74, 0xDD, 0xCE, 0x26, 0x57, 0xFA, 0xF0, 0x57

// Application Session K (u1_t[16]) in msb format
#define ABP_APPSKEY 0x5C, 0xB1, 0x85, 0xB0, 0x9E, 0x6E, 0x1D, 0xFE, 0x61, 0xAF, 0x2C, 0x59, 0x22, 0xB5, 0xD0, 0x58


#endif  // LORAWAN_KEYS_H_
