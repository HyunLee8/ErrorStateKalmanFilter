//from filter import ESKF, Data
//import pandas as pd

void RunFilter() {
    const data = Data();
    ESKF TrueFilter = new ESKF(data);
    while(TrueFilter.itteration < 5000) {
        TrueFilter.predict();
        TrueFilter.update();
    }
}