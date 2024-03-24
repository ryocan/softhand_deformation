#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "csvWholeFile");

    // get param
    std::string stringHeight;
    std::string pos; //Mid, TIp
    std::string segment;
    std::string obj;

    ros::NodeHandle nh;
    nh.getParam("csvWholeFile/segment", segment);
    nh.getParam("csvWholeFile/height", stringHeight);
    nh.getParam("csvWholeFile/pos", pos);
    nh.getParam("csvWholeFile/obj", obj);

    // folder name
    std::vector<int> bases = {91, 105, 119};
    // std::ofstream resultFile("/home/umelab-pgi5g/softhand_data/IROS2024/NoObj/result/result_seg" + segment + pos + ".csv");
    std::ofstream resultFile("/home/umelab-pgi5g/softhand_data/IROS2024/Obj/measurement/result/result_" + obj + pos + ".csv");


    for (int base : bases) {
        for (int index = 1; index <= 5; ++index) 
        {
            // std::string folder = "/home/umelab-pgi5g/softhand_data/IROS2024/NoObj/seg" + segment + "/" + std::to_string(base) + "_" + std::to_string(index);
            std::string folder = "/home/umelab-pgi5g/softhand_data/IROS2024/Obj/measurement/" + obj + "/" + std::to_string(base) + "_" + std::to_string(index);
            std::string filePath = folder + "/resultL.csv";
        
            std::ifstream inFile(filePath);
        
            if (!inFile.is_open()) {
                std::cerr << "Failed to open file: " << filePath << std::endl;
                continue;
            }

            std::string line;
            
            // midかtipによって飛ばす量が異なる
            std::getline(inFile, line);
            int num;
            if (pos == "Mid")
                num = 1;
            else if (pos == "Tip")
                num = 2;
            else
                return -1;

            for (int i = 0; i < num; i++)
                std::getline(inFile, line); // 最初の行を読み飛ばす

            std::stringstream ss(line);
            std::string cell;

            // Skip first 4 cells
            for (int i = 0; i < 4 && std::getline(ss, cell, ','); ++i) {}
            if (std::getline(ss, cell, ',')) {
                resultFile << cell << "," << std::endl;
            }
            inFile.close();
        }
    }
    resultFile.close();
    return 0;
}