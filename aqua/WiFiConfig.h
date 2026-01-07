#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <stddef.h>  // For size_t

struct HardcodedNetwork {
    const char* ssid;
    const char* password;
};

extern const HardcodedNetwork hardcodedNetworks[];
extern const size_t NUM_HARDCODED_NETWORKS;

#endif


/*
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

struct HardcodedNetwork {
    const char* ssid;
    const char* password;
};

// Extern declarations - tells compiler these exist somewhere
extern const HardcodedNetwork hardcodedNetworks[];
extern const size_t NUM_HARDCODED_NETWORKS;

#endif

*/