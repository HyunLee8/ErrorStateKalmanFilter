//entry point for running the filter

#include "eskf/utils/RunFilter.h"
#include "eskf/utils/Data.h"
#include "eskf/filter/filter.h"

void RunFilter() {
    const data = Data();
    ESKF TrueFilter = new ESKF(data);
    while(TrueFilter.itteration < 5000) {
        TrueFilter.predict();
        TrueFilter.update();
    }
}

