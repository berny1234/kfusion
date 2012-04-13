#include "KinectCamera.h"

using namespace xn;

KinectCamera::KinectCamera()
{
}

KinectCamera::~KinectCamera()
{
    context.StopGeneratingAll();

    image.Release();

    depth.Release();

    context.Shutdown();
    context.Release();
}

bool KinectCamera::Connect(int index)
{
    m_index = index;

	XnStatus rc;
	EnumerationErrors errors;
    rc = context.Init();

	NodeInfoList list;
	rc = context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, list, &errors);

	int i = 0;
	for (NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it, ++i)
	{
        if(i == index)
        {
            printf("%d\n", i);
            NodeInfo deviceNodeInfo = *it;
		    context.CreateProductionTree(deviceNodeInfo);
            rc |= depth.Create(context);
            rc |= image.Create(context);
            
            XnMapOutputMode m;
            m.nXRes = 640;
            m.nYRes = 480;
            image.SetMapOutputMode(m);

            rc |= context.StartGeneratingAll();
            break;
        }
    }

    if(i != index)
    {
        return false;
    }

    printf("Success: %d\n", rc);

	return true;
}

void KinectCamera::GetImage(uchar4 *out)
{
	//TODO
	/*HRESULT hr;
	NUI_IMAGE_FRAME frame;
	NUI_LOCKED_RECT rect;

	hr = m_instance->NuiImageStreamGetNextFrame(m_hImage, 100, &frame);
	if FAILED(hr) return;
	
	frame.pFrameTexture->LockRect(0, &rect, NULL, 0);
	for (int y = 0; y < 480; y++)
	{
		const uchar4* src = reinterpret_cast<const uchar4*>(rect.pBits + y * rect.Pitch);
		uchar4* dst = out + y * 640;
		for (int x = 0; x < 640; x++)
		{
			uchar4 pixel = src[x];
			std::swap(pixel.x, pixel.z);
			dst[640-1-x] = pixel; // unmirror
		}
	}

	frame.pFrameTexture->UnlockRect(0);
	m_instance->NuiImageStreamReleaseFrame(m_hImage, &frame);		*/ 
}

std::string KinectCamera::Serial()
{
    //NOT POSSIBLE WITH OPENNI!

    /*NodeInfoList list;
    context.EnumerateExistingNodes(list, XN_NODE_TYPE_DEVICE);

    NodeInfo deviceNodeInfo = *(list.Begin());

    const XnChar* name = deviceNodeInfo.GetDescription().strName;*/

    char buff[256];
    itoa(m_index, buff, 10);

	return std::string(buff);
}
