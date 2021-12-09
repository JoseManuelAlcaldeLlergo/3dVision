#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "scanning_pattern_sequence.hpp"

namespace fsiv {

ScanningPatternSequence::~ScanningPatternSequence()
{}

void
show_scanning(const std::shared_ptr<ScanningPatternSequence>& scan)
{
    int i=0;
    int key=-1;
           std::ostringstream wname;
    while(key!=27)
    {
        wname.str("");
        wname << "Scann "<<i << " (<-, ->, Esc=salir)";
        cv::namedWindow(wname.str(), cv::WINDOW_AUTOSIZE+ cv::WINDOW_KEEPRATIO);
        cv::imshow(wname.str(), scan->seq[i]);
        key = cv::waitKey(0) & 0xff;
        cv::destroyWindow(wname.str());
        if (key == 81)
            i = (i-1 + int(scan->seq.size())) % int(scan->seq.size());
        else if (key == 83)
            i = (i+1) % int(scan->seq.size());
    }
}


} //namespace fsiv
