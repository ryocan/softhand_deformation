#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "csv");

    // get param
    std::string stringHeight;
    std::string pos; //Mid, TIp
    std::string segment;
     

    ros::NodeHandle nh;
    nh.getParam("csv/segment", segment);
    nh.getParam("csv/height", stringHeight);
    nh.getParam("csv/pos", pos);

    std::vector<std::string> folders = {stringHeight + "_1", stringHeight + "_2", stringHeight + "_3", stringHeight + "_4", stringHeight + "_5"};
    std::ofstream resultFile("/home/umelab-pgi5g/softhand_data/IROS2024/NoObj/result/result_seg" + segment + "_" + stringHeight + pos +"L.csv", std::ios::app); // 結果を追記モードで開く

    if (!resultFile.is_open()) {
        std::cerr << "result.csvを開けませんでした。" << std::endl;
        return -1;
    }

    for (const auto& folder : folders) {
        std::string filePath = "/home/umelab-pgi5g/softhand_data/IROS2024/NoObj/seg" + segment +"/" + folder + "/resultL.csv";
        std::ifstream inputFile(filePath);

        if (!inputFile.is_open()) {
            std::cerr << folder << "/input.csvを開けませんでした。" << std::endl;
            continue;
        }

        std::string line;
        
        // midかtipによって飛ばす量が異なる
        int num;
        if (pos == "Mid")
            num = 1;
        else if (pos == "Tip")
            num = 2;
        else
            return -1;

        for (int i = 0; i < num; i++)
            std::getline(inputFile, line); // 最初の行を読み飛ばす


        if (std::getline(inputFile, line)) { // 最初の行を読み込む
            std::istringstream iss(line);
            std::string value;
            int colCount = 0;

            while (std::getline(iss, value, ','))
            {
                if(colCount == 4)
                    resultFile << value << "," << "\n";
                colCount++;
            }
        }

        inputFile.close();
    }

    resultFile.close();
    std::cout << "Fin" << std::endl;

    return 0;
}