#include "view.hpp"

namespace fsiv {
const cv::Mat&
View::iimg() const
{
    return _iimg_fg;
}

const std::string&  View::id () const
{
  return _id;
}

const cv::Mat&
View::colored_view (void) const
{
  return _view;
}

void
View::set_colored_view (const cv::Mat& new_view)
{
    _view = new_view;
}

const cv::Mat&
View::foreground (void) const
{
  return _foreground;
}

} // namespace fsiv
