"""
Author: Isaac Lee
Date: Jan 10, 2026
Description:
    I wanted the main method to be squeaky
    clean having only one function call so 
    I made a secondary entry point here :O.
"""

#include "eskf/utils/RunFilter.h"
#include "eskf/utils/Data.h"
#include "eskf/filter/filter.h"
#include <iostream>

void RunFilter() {
    const data = Data();
    ESKF TrueFilter = new ESKF(data);
    while(TrueFilter.itteration < 5000) {
        TrueFilter.predict();
        TrueFilter.update();
    }
}

