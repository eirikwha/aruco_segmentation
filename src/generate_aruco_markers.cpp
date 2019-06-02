//
// Created by eirik on 11.05.19.
//
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv-3.3.1-dev/opencv2/imgproc.hpp>

using namespace cv;

namespace {
    const char* about = "Generate ArUco marker images. NB! Only marker IDs from 0 to 50 will be generated.";
    const char* keys  =
            "{@outfolder    |       | Output folder }"
            "{d             |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
            "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
            "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
            "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
            "{num           |       | Number of markers to be generated }";
}


int main(int argc, char *argv[]) {
    srand (time(NULL));
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 4) {
        parser.printMessage();
        return 0;
    }

    // TODO: Set default dictionary
    // TODO: If generator params are not given, use some default size
    // TODO: Input dictionary and number of markers, also save type of marker? Maybe not neccessary
    int dictionaryId = parser.get<int>("d");
    int numMarkers = parser.get<int>("num");

    if (numMarkers > 49){
        numMarkers = 49;
    }

    String out = parser.get<String>(0);

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    int markerId;
    for (int i = 0; i < numMarkers; i++){
        cv::Mat markerImg;
        int tmpMarkerId = markerId;
        markerId = rand() % 49;

        while (markerId == tmpMarkerId){
            markerId = rand() % 49;
            return markerId;
        }

        aruco::drawMarker(dictionary, markerId, 200, markerImg, 1);

        imshow("marker", markerImg);
        waitKey(0);
        imwrite(out + "dictID_" + std::to_string(dictionaryId) + std::string("_markerID_") + std::to_string(markerId) + std::string(".png"), markerImg);
    }

    return 0;
}

