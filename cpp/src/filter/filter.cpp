#include <filter.h>
#include <data.h>
#include <Eigen/Dense>
#include <vector>

class Data {
    private:
        TimeCol = 0;
        AccXCol = 1;
        AccYCol = 2;
        AccZCol = 3;
        GyroXCol = 4;
        GyroYCol = 5;
        GyroZCol = 6;
    
        PosXCol = 0;
        PosYCol = 1;
        PosZCol = 2;
        VelXCol = 3;
        VelYCol = 4;
        VelZCol = 5;
    
        sensor_data = readCSV('sensor_data.csv');
        motion_data = readCSV('motion_data.csv');
    public:
    }