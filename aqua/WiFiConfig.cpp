#include "WiFiConfig.h"

const HardcodedNetwork hardcodedNetworks[] = {
    {"realme 5g", "bhavil23!"},
    {"ishwar", "12345689"},
    {"note 5pro", "bhavil23!"},
    {"lenovo", "12345678"}  
};

const size_t NUM_HARDCODED_NETWORKS = sizeof(hardcodedNetworks) / sizeof(hardcodedNetworks[0]);


/*
#include "WiFiConfig.h"

// Actual definitions - only in ONE file
const HardcodedNetwork hardcodedNetworks[] = {
    {"realme 5g", "bhavil23!"},
    {"ishwar", "12345689"},
    {"note 5pro", "bhavil23!"},
    {"lenovo", "12345678"}  
};

const size_t NUM_HARDCODED_NETWORKS = sizeof(hardcodedNetworks) / sizeof(hardcodedNetworks[0]);
*/