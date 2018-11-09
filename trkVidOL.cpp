#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include "utils/tags.h"
#include "utils/datfile.h"

using namespace cv;
using namespace std;

const char* params
        = "{ help h   | | Print usage }"
          "{ trkVid v | | Path to a tracking video }"
          "{ fDat d   | | Path to a .dat file }"
          "{ fTags t  | | Path to a .tags file }";

int main(int argc, char** argv ) {
        CommandLineParser parser(argc, argv, params);
        parser.about("Program to highlight tracking video with tracking data (.tags and .dat files)");
        if (parser.has("help") || !parser.has("fDat") || !parser.has("fTags") || !parser.has("trkVid")) {
                parser.printMessage();
                return 1;
        }

        VideoCapture capture(parser.get<String>("trkVid"));
        if (!capture.isOpened()) {
                cerr << "Unable to open: " << parser.get<String>("trkVid") << endl;
                return 1;
        }

        DatFile dat;
        if (!dat.exists(parser.get<String>("fDat"))) {
                cerr << "Dat file does not exist" << endl;
                return 1;
        }
        dat.open(parser.get<String>("fDat"), true);

        framerec datFrame;
        Mat vidFrame;

        double scW = (capture.get(CAP_PROP_FRAME_WIDTH)) / ((double) IMAGE_WIDTH);
        double scH = (capture.get(CAP_PROP_FRAME_HEIGHT)) / ((double) IMAGE_HEIGHT);

        dat.read_frame(datFrame);
        capture >> vidFrame;

        int queenId = 224;

        namedWindow("Current frame", WINDOW_AUTOSIZE);
        do {
                if (datFrame.frame > 964970) {
                        capture >> vidFrame;
                        string frameNumb = to_string(datFrame.frame);
                        putText(vidFrame, frameNumb, Point(0, 50), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, Scalar(0,0,255), 2, LINE_8, false);
                        for (int tagNo = 0; tagNo < tag_count; tagNo++) {
                                if (datFrame.tags[tagNo].x >= 0) {
                                        double x = (double)datFrame.tags[tagNo].x * scH;
                                        double y = (double)datFrame.tags[tagNo].y * scW;
                                        string idString = to_string(tagNo);
                                        if (tagNo == queenId) {
                                                circle(vidFrame, Point(x, y), 2, Scalar(0,255,255), 2);
                                        } else {

                                                circle(vidFrame, Point(x, y), 2, Scalar(0,0,255), 2);
                                        }
                                        putText(vidFrame, idString, Point(x + 4.0, y + 4.0), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, Scalar(0,255,0), 1, LINE_8, false);

                                }
                        }
                        imshow("Current frame", vidFrame);
                        waitKey(1000);
                }

        } while (!vidFrame.empty() && dat.read_frame(datFrame));

        return 0;
}
