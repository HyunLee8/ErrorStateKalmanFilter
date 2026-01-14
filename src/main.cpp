/*
Author: Isaac Lee
Date: Jan 10, 2026
Index:

    /data       - preprocessing entry point for fetching data as Eigen::Matrix and
                  carries all data.

    /filter     - Entire Error State Kalman Filter logic is in here -> Explanations 
                  provided in filter.cpp

    /include    - .h header files for filter, utils, and data

    /utils      - support functions for fetching sensor and motion data as well
                  as running the filter after main entry point

*/

#include "eskf/utils/RunFilter.h"
#include <iostream>

int main() {
    RunFilter();
    return 0;
}
