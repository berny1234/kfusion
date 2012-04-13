#include <windows.h>
#include <vector_types.h>
#include <string>
#include <XnCppWrapper.h>

#ifndef KINECT_CAM
#define KINECT_CAM

namespace nearfar
{
	class Camera;
}

class KinectCamera
{
public:
	KinectCamera();
	~KinectCamera();
	bool Connect(int index);
	void GetImage(uchar4* out);	
	std::string Serial();
    int m_index;
    xn::Context context;
    xn::DepthGenerator depth;
    xn::ImageGenerator image;
protected:
};

#endif