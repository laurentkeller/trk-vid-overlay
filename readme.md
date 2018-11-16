## Tracking Video Visualizer

# Description
Program to overlay tracking information on a tracking video.
Inputs:
* Tracking video `boxXX-YYYYMMDD-HHMM.avi`
* Dat file `filename.dat`
* Tags file `filename.tags`
* Interaction list file `filename.txt` (optional)

Output:
* Video with overlayd tracking information `result.avi`

# Dependencies
* tpppl Tracking Post Processing Pipeline Library
* OpenCV Version > 3
* cmake > 3.6

# Usage
Get command line arguments usage: `./trkVidOL -h`

# TODOs
* Add triangle and head with stuff
* Add activity information (whatever it is)
* Make better interface (maybe a config file)
* Add frame detection from input video stream
* Remove debug frames printed on screen
