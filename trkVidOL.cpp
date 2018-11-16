#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>
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
          "{ fInteract i  | | Path to an interaction (.txt) file }"
          "{ show s       | | Show video preview }";

// structure of an interaction
struct data {
        double time_start;
        double time_stop;
        uint32_t frame_start;       // frame of interaction
        uint32_t frame_stop;        // frame of end of interaction (filled only during filtering)
        uint16_t box;
        //coor ant 1
        uint16_t x1;
        uint16_t y1;
        int16_t a1;
        // coor ant 2
        uint16_t x2;
        uint16_t y2;
        int16_t a2;
        int det;
        int direction;     // direction of interaction (1: ant1 interacts, 2: ant2 interacts, 3:both ants interact);
};

// INteraction event
struct event {
        uint16_t tag1;
        uint16_t tag2;
        data d;
        char s;       // state of interaction: long, blinking
};

typedef vector <data> interactions;
typedef vector <vector <int> > matrice;

// convert a tag in the corresponding index of the tag_list table
bool find_idx(const int tag, int& idx){
        int i(0);
        do {
                if (tag == tag_list[i]) {
                        idx = i;
                        return true;
                }
                i++;
        } while(idx == -1 && i<tag_count);
        return false;
}

// function to compare elements of vector
bool cmp(const event& a, const event& b){
        return a.d.frame_start < b.d.frame_start;
}

/** void read_interaction_file(char* filename, vector <vector <interactions> >& i_table)
 * \brief Opens input file with list of interactions (expected format is frame,box,IDant1,IDant2,x1,y1,a1,x2,y2,a2)
 *		reads them into a 3 dimensional vector
 * \param filename Name of the input file to read
 * \param i_table Table of interactions to fill
 */
void read_interaction_file(string filename, vector <vector <interactions> >& i_table){
        ifstream f;
        f.open(filename.c_str());
        if (!f.is_open()) {
                cerr << "CANNOT_OPEN_FILE " << filename << endl;

        }

        string s;
        getline(f,s);
        while (!f.eof()) {
                getline(f,s);
                stringstream ss;
                ss.str(s);
                data temp;
                memset(&temp, 0, sizeof(temp));
                temp.frame_stop = 0;
                if (!ss.fail()) {
                        int tag1;
                        ss >> tag1;
                        ss.ignore(1,',');
                        int tag2;
                        ss >> tag2;
                        ss.ignore(1,',');
                        ss>>temp.frame_start;
                        ss.ignore(1,',');
                        ss>>temp.frame_stop;
                        ss.ignore(1,',');
                        ss>>temp.time_start;
                        ss.ignore(1,',');
                        ss>>temp.time_stop;
                        ss.ignore(1,',');
                        ss>>temp.box;
                        ss.ignore(1,',');
                        ss>>temp.x1;
                        ss.ignore(1,',');
                        ss>>temp.y1;
                        ss.ignore(1,',');
                        ss>>temp.a1;
                        ss.ignore(1,',');
                        ss>>temp.x2;
                        ss.ignore(1,',');
                        ss>>temp.y2;
                        ss.ignore(1,',');
                        ss>>temp.a2;
                        ss.ignore(1,',');
                        ss>>temp.direction;
                        ss.ignore(1,',');
                        ss>>temp.det;

                        // cout << "temp: "<< tag1 << " " << tag2 << " " << temp.frame_start <<
                        // " " << temp.frame_stop << " " << temp.time_start << " " << temp.time_stop <<
                        // " " << temp.box << " " << temp.x1 << " " << temp.y1 << " " << temp.a1 <<
                        // " " << temp.x2 << " " << temp.y2 << " " << temp.a2 <<
                        // " " << temp.direction << " " << temp.det << endl;
                        // cout<<"read: "<< s << endl;

                        // find index of each tag
                        int idx1 (-1);
                        int idx2 (-1);
                        if (!find_idx(tag1, idx1)) {
                                stringstream ss;
                                ss << tag1;
                                string info = ss.str();
                                cerr << "TAG_NOT_FOUND" << info << endl;
                        }
                        if (!find_idx(tag2, idx2)) {
                                stringstream ss;
                                ss << tag2;
                                string info = ss.str();
                                cerr << "TAG_NOT_FOUND" << info << endl;
                        }

                        // add interaction to list (matrix is symmetric)
                        i_table[idx1][idx2].push_back(temp);
                        i_table[idx2][idx1].push_back(temp);
                }
        }
        f.close();
}

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

        vector <vector <interactions> > i_table;
        i_table.resize(tag_count);
        for (int i = 0; i < tag_count; i++) {
                i_table[i].resize(tag_count);
        }
        bool interactions = false;
        if (parser.has("fInteract")) {
                interactions = true;
                read_interaction_file(parser.get<string>("fInteract"), i_table);
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

        if (parser.has("show")) {
                namedWindow("Current frame", WINDOW_AUTOSIZE);
                namedWindow("roi1", WINDOW_AUTOSIZE);
                namedWindow("roi2", WINDOW_AUTOSIZE);
                namedWindow("roi3", WINDOW_AUTOSIZE);
                namedWindow("roi4", WINDOW_AUTOSIZE);
                namedWindow("roi5", WINDOW_AUTOSIZE);
                namedWindow("roi6", WINDOW_AUTOSIZE);
                namedWindow("roi7", WINDOW_AUTOSIZE);
                namedWindow("roi8", WINDOW_AUTOSIZE);
        }
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
                                                                line(vidFrame, Point(xHead, yHead), Point(xTail, yTail), Scalar(255,255,0), 1, LINE_8);
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

                        if (interactions) {
                                for (int i = 0; i < tag_count - 1; i++) {
                                        for (int j = i + 1; j < tag_count; j++) {
                                                if (!i_table[i][j].empty()) {
                                                        vector<data>::iterator it = i_table[i][j].begin();
                                                        while(it != i_table[i][j].end()) {
                                                                if (it->frame_start <= datFrame.frame && it->frame_stop >= datFrame.frame) {
                                                                        line(vidFrame, Point(it->x1 * scH, it->y1 * scW), Point(it->x2 * scH, it->y2 * scW), Scalar(0,215,255), 1, LINE_8);
                                                                }
                                                                it++;
                                                        }
                                                }
                                        }
                                }
                        }

                        outputVid << vidFrame;
                        if (parser.has("show")) {

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
                                imshow("Current frame", vidFrame);
                                waitKey(1000);
                        }
                }

        } while (!vidFrame.empty() && fdat.read_frame(datFrame));

        return 0;
}
