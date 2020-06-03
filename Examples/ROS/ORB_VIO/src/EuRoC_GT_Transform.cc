/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>

using namespace std;

struct Data
{
    Data(FILE *f)
    {
        fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az);
        t /= 1e9;
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};

int main(int argc, char **argv)
{
    std::cout << "EuRoC GroundTruth Transform into TUM format ..." << endl;

    if(argc != 2)
    {
        std::cerr << "\033[31m" << "Usage: ./EuRoC_GT_Transform dataset_name" << "\033[0m" << std::endl;
        return 0;
    }
    string sequence_name = argv[1];
    string csv_file = "./GroundTruth_EuRoC/" + sequence_name + "/data.csv";

    std::cout << "\033[32m" << "load ground truth " << csv_file << "\033[0m" << std::endl;
    FILE *f = fopen(csv_file.c_str(), "r");
    if (f==NULL)
    {
      ROS_WARN("can't load ground truth; wrong path");
//      std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
      return 0;
    }
    ROS_INFO_STREAM("load ground truth");
    char tmp[10000];
    fgets(tmp, 10000, f);

    vector<Data> benchmark;
    while (!feof(f))
    {
        benchmark.emplace_back(f);
    }
    fclose(f);
    benchmark.pop_back();
    ROS_INFO("Data loaded: %d", (int)benchmark.size());

    for(size_t i=0; i<10; i++)
    {
        Data data = benchmark[i];
        ROS_INFO("timestamp = %f, position = [%f, %f, %f]", data.t, data.px, data.py, data.pz);
    }

    string gt = "./GroundTruth_EuRoC/" + sequence_name + "/groundtruth.txt";

    ofstream f_write;
    f_write.open(gt.c_str());
    f_write << fixed;

    f_write << "# ground truth trajectory" << endl;
    f_write << "# file: '" + sequence_name + ".bag'" << endl;
    f_write << "# timestamp tx ty tz qx qy qz qw" << endl;
    for(size_t i=0; i<benchmark.size(); i++)
    {
        Data data = benchmark[i];
        f_write << setprecision(6) << data.t << setprecision(7) << " ";
        f_write << data.px << " " << data.py << " " << data.pz << " ";
        f_write << data.qx << " " << data.qy << " " << data.qz << " " << data.qw << " ";

        f_write << endl;
    }



     f_write.close();
     cout << "EuRoC GroundTruth successfully transform into TUM format. " << endl;

    return 0;
}


