#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>

std::pair<double, double> calculateMeanStdDev(const std::string& fileName) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << fileName << " を開けませんでした。" << std::endl;
        throw std::runtime_error("ファイルを開けませんでした。");
    }

    std::vector<double> values;
    double sum = 0.0;
    std::string sValue;
    double value;
    int count = 0;

    while (file >> sValue) {
        value = std::stod(sValue);
        values.push_back(value);
        sum += value;
        ++count;
    }

    if (count < 2) {
        throw std::runtime_error("サンプルサイズが小さすぎます。");
    }

    double mean = sum / count;
    double sumDiffSq = 0.0;

    for (double val : values) {
        sumDiffSq += (val - mean) * (val - mean);
    }

    double variance = sumDiffSq / (count - 1); // 不偏分散
    double stddev = std::sqrt(variance);

    return std::make_pair(mean, stddev);
}


int main() {
    std::vector<int> segments = {2, 10, 20};
    // std::vector<int> numbers = {75, 95, 115};
    std::vector<std::string> objs = {"1circle", "2rectangle", "3clover", "4star", "5triangleA"};
    std::vector<std::string> types = {"Mid", "Tip"};
    // std::string path = "/home/umelab-pgi5g/softhand_data/IROS2024/NoObj/result/";
    std::string path = "/home/umelab-pgi5g/softhand_data/IROS2024/Obj/measurement/result/";
    std::vector<std::string> fileNames;

    for(const std::string& obj : objs) {
        for (const std::string& type : types) {
            std::ostringstream fileName;
            fileName << path << "result_" << obj << type << ".csv";
            std::cout << "Generated filename: " << fileName.str() << std::endl;

            fileNames.push_back(fileName.str());
        }
    }

    // ファイル名の生成
    // for (int seg : segments) {
    //     for (int num : numbers) {
    //         for (const std::string& type : types) {
    //             std::ostringstream fileName;
    //             // fileName << path << "result_seg" << seg << "_" << num << type << "R.csv";
    //             fileName << path << "result_seg" << seg << type << ".csv";
    //             fileNames.push_back(fileName.str());
    //         }
    //     }
    // } 


    // std::ofstream resultFile("/home/umelab-pgi5g/softhand_data/IROS2024/NoObj/result/resultMeanStddiv.csv");
    std::ofstream resultFile("/home/umelab-pgi5g/softhand_data/IROS2024/Obj/measurement/result/resultMeanStddiv.csv");
    if (!resultFile.is_open()) {
        std::cerr << "result.csv を開けませんでした。" << std::endl;
        return -1;
    }

    resultFile << ",mean,stddev\n"; // ヘッダー行

    for (const auto& fileName : fileNames) 
    {
        try 
        {
            auto [mean, stddev] = calculateMeanStdDev(fileName);

            // ファイル名の処理
            std::size_t startPos = fileName.find("result");
            std::size_t endPos = fileName.find(".csv");
            std::string processedName = fileName.substr(startPos, endPos - startPos);

            resultFile << processedName << "," << mean << "," << stddev << "\n";
        } 
        catch (const std::exception& e) 
        {   
            std::cerr << "エラーが発生しました: " << e.what() << std::endl;
            continue; // エラーがあれば次のファイルに移動
        }
    }

    resultFile.close();
    return 0;
}