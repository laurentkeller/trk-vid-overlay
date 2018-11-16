#include <iostream>
#include <string>
#include <cmath>
#include <deque>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include "tpppl/tags3.h"
#include "tpppl/datfile.h"

#include "numPatterns.hpp"

using namespace cv;
using namespace std;

const int tl = 10; // Length of the trajectory printed in the video

const char* params
        = "{ help h       | | Print usage }"
          "{ trkVid v     | | Path to a tracking video }"
          "{ fDat d       | | Path to a .dat file }"
          "{ fTags t      | | Path to a .tags file }"
          "{ fInteract i  | | Path to an interaction (.txt) file }";

int main(int argc, char** argv ) {
        CommandLineParser parser(argc, argv, params);
        parser.about("Program to highlight tracking video with tracking data (.tags and .dat files)");
        if (parser.has("help") || !parser.has("fDat") || !parser.has("fTags") || !parser.has("trkVid")) {
                cout << CV_MAJOR_VERSION << " " << CV_MINOR_VERSION << endl;
                parser.printMessage();
                return 1;
        }

        VideoCapture capture(parser.get<String>("trkVid"));
        if (!capture.isOpened()) {
                cerr << "Unable to open: " << parser.get<String>("trkVid") << endl;
                return 1;
        }

        VideoWriter outputVid;
        #if CV_MAJOR_VERSION >= 4
        int codec = outputVid.fourcc('D', 'I', 'V', '3');
        #else
        int codec = CV_FOURCC('D', 'I', 'V', '3');
        #endif
        outputVid.open("result.avi", codec, capture.get(CAP_PROP_FPS), Size((int)capture.get(CAP_PROP_FRAME_WIDTH), (int)capture.get(CAP_PROP_FRAME_HEIGHT)), true);
        if (!outputVid.isOpened()) {
                cout << "Could not open the output video for write" << endl;
                return -1;
        }

        DatFile fdat;
        if (!fdat.exists(parser.get<String>("fDat"))) {
                cerr << "fdat file does not exist" << endl;
                return 1;
        }
        fdat.open(parser.get<String>("fDat"), true);

        TagsFile ftag;
        string fname_s = parser.get<String>("fTags");
        char fname_c[fname_s.size() + 1];
        strcpy(fname_c, fname_s.c_str());
        ftag.read_file(fname_c);

        int frameOfDeath[1024];
        for (int i = 0; i < 1024; i++) {
                frameOfDeath[i] = INT_MAX;
        }

        for (int i = 0; i < tag_count; i++) {
                if (ftag.get_state(i) && ftag.get_death(i) > 0) {
                        frameOfDeath[ftag.get_tag(i)] = ftag.get_death(i);
                }
        }

        framerec datFrame;
        deque<framerec> qDatFrames;
        Mat vidFrame;

        double scW = (capture.get(CAP_PROP_FRAME_WIDTH)) / ((double) IMAGE_WIDTH);
        double scH = (capture.get(CAP_PROP_FRAME_HEIGHT)) / ((double) IMAGE_HEIGHT);

        fdat.read_frame(datFrame);
        capture >> vidFrame;

        int queenId = 665;
        double hil = 5.0; // Heading indicator length

        namedWindow("Current frame", WINDOW_AUTOSIZE);
        namedWindow("roi1", WINDOW_AUTOSIZE);
        namedWindow("roi2", WINDOW_AUTOSIZE);
        namedWindow("roi3", WINDOW_AUTOSIZE);
        namedWindow("roi4", WINDOW_AUTOSIZE);
        namedWindow("roi5", WINDOW_AUTOSIZE);
        namedWindow("roi6", WINDOW_AUTOSIZE);
        namedWindow("roi7", WINDOW_AUTOSIZE);
        namedWindow("roi8", WINDOW_AUTOSIZE);

        do {
                if (datFrame.frame > 964970) {
                        capture >> vidFrame;
                        qDatFrames.push_front(datFrame);
                        if (qDatFrames.size() > tl - 1) {
                                qDatFrames.pop_back();
                        }
                        string frameNumb = to_string(datFrame.frame);
                        putText(vidFrame, frameNumb, Point(0, 50), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, Scalar(0,0,255), 2, LINE_8, false);
                        for (int tagNo = 0; tagNo < tag_count; tagNo++) {
                                if (datFrame.tags[tagNo].x >= 0) {
                                        // Draw the trajectory first
                                        double xHead = -1.0;
                                        double xTail = -1.0;
                                        double yHead = -1.0;
                                        double yTail = -1.0;
                                        for (int i = 0; i < tl; i++) {
                                                if (i > qDatFrames.size() - 1) {
                                                        break;
                                                }
                                                if (qDatFrames.at(i).tags[tagNo].x >= 0) {
                                                        xTail = xHead;
                                                        yTail = yHead;
                                                        xHead = (double)qDatFrames.at(i).tags[tagNo].x * scH;
                                                        yHead = (double)qDatFrames.at(i).tags[tagNo].y * scW;
                                                        if (xTail > -1.0) {
                                                                line(vidFrame, Point(xHead, yHead), Point(xTail, yTail), Scalar(255, 255, 0), 1, LINE_8);
                                                        }
                                                }
                                        }

                                        // Now draw everything else
                                        double x = (double)datFrame.tags[tagNo].x * scH;
                                        double y = (double)datFrame.tags[tagNo].y * scW;
                                        string idString = to_string(tag_list[tagNo]);
                                        if (tag_list[tagNo] == queenId) {
                                                circle(vidFrame, Point(x, y), 2, Scalar(0,255,255), 2);
                                        } else {
                                                if (datFrame.frame < frameOfDeath[tag_list[tagNo]]) {
                                                        circle(vidFrame, Point(x, y), 2, Scalar(0,0,255), 2);
                                                } else {
                                                        circle(vidFrame, Point(x, y), 2, Scalar(255,0,255), 2);
                                                }
                                        }
                                        putText(vidFrame, idString, Point(x + 4.0, y + 4.0), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, Scalar(0,255,0), 1, LINE_8, false);
                                        line(vidFrame, Point(x, y), Point(x + hil * cos((double)datFrame.tags[tagNo].a * M_PI / 180.0 / 100.0), y + hil * sin((double)datFrame.tags[tagNo].a * M_PI / 180.0 / 100.0)), Scalar(255, 0, 0), 1, LINE_8);
                                }
                        }
                        Rect roi_tot = Rect(53, 2, 73, 10);
                        Mat image_roi = vidFrame(roi_tot);
                        Mat roi[8];
                        int len = 9;
                        int xOffset = 54;
                        int yOffset = 2;
                        int width = 7;
                        int height = 10;
                        for (int i = 0; i < 8; i++) {
                                roi[i] = vidFrame(Rect(xOffset + i * len, yOffset, width, height));
                        }

                        outputVid << vidFrame;
                        imshow("Current frame", image_roi);
                        imshow("roi1", roi[0]);
                        imshow("roi2", roi[1]);
                        imshow("roi3", roi[2]);
                        imshow("roi4", roi[3]);
                        imshow("roi5", roi[4]);
                        imshow("roi6", roi[5]);
                        imshow("roi7", roi[6]);
                        imshow("roi8", roi[7]);

                        double worb[width * height];
                        for (int j = yOffset; j < yOffset + 10; j++) {
                          bool isNum[10] = {false,false,false,false,false,false,false,false,false,false};
                                for (int i = 3 * xOffset; i < 3 * xOffset + 8 * len * 3; i++) {
                                        Scalar colour = vidFrame.at<uchar>(j, i);
                                        if (colour[0] == 255) {
                                                cout << 1 << ", ";

                                        } else if (colour[0] == 0) {
                                                cout << "0" << ", ";

                                        } else {
                                                cout << 5 << ", ";
                                        }
                                }
                                cout << endl;
                        }
                        waitKey(1000);
                }

        } while (!vidFrame.empty() && fdat.read_frame(datFrame));

        return 0;
}
