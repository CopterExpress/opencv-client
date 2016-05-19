#ifndef _HEADER_2QDEJH8631GB8AP7QJ5C_INCLUDED_
#define _HEADER_2QDEJH8631GB8AP7QJ5C_INCLUDED_

namespace cv { class Mat; }
namespace copexp
{

class RC2Queue;

void cameraOpen(RC2Queue &queue);

void cameraProcess(RC2Queue& queue, cv::Mat& mat);

} // namespace copexp

#endif // _HEADER_2QDEJH8631GB8AP7QJ5C_INCLUDED_
