//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOS.h>
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <math.h>
#include <vector>

#include <XnCppWrapper.h>
using namespace xn;

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "SamplesConfig.xml"

#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024

#define DISPLAY_MODE_OVERLAY	1
#define DISPLAY_MODE_DEPTH		2
#define DISPLAY_MODE_IMAGE		3
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MAX_DEPTH 10000

#ifndef GL_SGIS_generate_mipmap
#define GL_GENERATE_MIPMAP_SGIS           0x8191
#define GL_GENERATE_MIPMAP_HINT_SGIS      0x8192
#endif

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
float g_pDepthHist[MAX_DEPTH];
std::vector<XnRGB24Pixel*> g_pTexMaps;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;

unsigned int g_nViewState = DEFAULT_DISPLAY_MODE;

std::vector<Context> g_contexts;
//Context g_context;
std::vector<ScriptNode> g_scriptNodes;
//DepthGenerator g_depth;
std::vector<DepthGenerator> g_depths;
std::vector<ImageGenerator> g_images;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

void glutIdle (void)
{
	// Display the frame
	glutPostRedisplay();
}

void glutDisplay (void)
{
	XnStatus rc = XN_STATUS_OK;
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);


	for(int i = 0; i < g_contexts.size(); i++)
	{
		
	// Read a new frame
	rc = g_contexts[i].WaitOneUpdateAll(g_depths[i]);
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return;
	}

	DepthMetaData l_depthMD;
	ImageMetaData l_imageMD;

	g_depths[i].GetMetaData(l_depthMD);
	g_images[i].GetMetaData(l_imageMD);

	const XnDepthPixel* pDepth = l_depthMD.Data();
	const XnUInt8* pImage = l_imageMD.Data(); 

	unsigned int nImageScale = GL_WIN_SIZE_X / l_depthMD.FullXRes();

	// Copied from SimpleViewer
	// Clear the OpenGL buffers
	
	// Calculate the accumulative histogram (the yellow display...)
	xnOSMemSet(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));

	unsigned int nNumberOfPoints = 0;
	for (XnUInt y = 0; y < l_depthMD.YRes(); ++y)
	{
		for (XnUInt x = 0; x < l_depthMD.XRes(); ++x, ++pDepth)
		{
			if (*pDepth != 0)
			{
				g_pDepthHist[*pDepth]++;
				nNumberOfPoints++;
			}
		}
	}
	for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}

	xnOSMemSet(g_pTexMaps[i], 0, g_nTexMapX*g_nTexMapY*sizeof(XnRGB24Pixel));

	// check if we need to draw image frame to texture
	if (g_nViewState == DISPLAY_MODE_OVERLAY ||
		g_nViewState == DISPLAY_MODE_IMAGE)
	{
		const XnRGB24Pixel* pImageRow = l_imageMD.RGB24Data();
		XnRGB24Pixel* pTexRow = g_pTexMaps[i] + l_imageMD.YOffset() * g_nTexMapX;

		for (XnUInt y = 0; y < l_imageMD.YRes(); ++y)
		{
			const XnRGB24Pixel* pImage = pImageRow;
			XnRGB24Pixel* pTex = pTexRow + l_imageMD.XOffset();

			for (XnUInt x = 0; x < l_imageMD.XRes(); ++x, ++pImage, ++pTex)
			{
				*pTex = *pImage;
			}

			pImageRow += l_imageMD.XRes();
			pTexRow += g_nTexMapX;
		}
	}

	// check if we need to draw depth frame to texture
	if (g_nViewState == DISPLAY_MODE_OVERLAY ||
		g_nViewState == DISPLAY_MODE_DEPTH)
	{
		const XnDepthPixel* pDepthRow = l_depthMD.Data();
		XnRGB24Pixel* pTexRow = g_pTexMaps[i] + l_depthMD.YOffset() * g_nTexMapX;

		for (XnUInt y = 0; y < l_depthMD.YRes(); ++y)
		{
			const XnDepthPixel* pDepth = pDepthRow;
			XnRGB24Pixel* pTex = pTexRow +l_depthMD.XOffset();

			for (XnUInt x = 0; x < l_depthMD.XRes(); ++x, ++pDepth, ++pTex)
			{
				if (*pDepth != 0)
				{
					int nHistValue = g_pDepthHist[*pDepth];
					pTex->nRed = nHistValue;
					pTex->nGreen = nHistValue;
					pTex->nBlue = 0;
				}
			}

			pDepthRow += l_depthMD.XRes();
			pTexRow += g_nTexMapX;
		}
	}

	// Create the OpenGL texture map
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, g_pTexMaps[i]);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glBegin(GL_QUADS);

	int nXRes = l_depthMD.FullXRes();
	int nYRes = l_depthMD.FullYRes();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f( (i*(GL_WIN_SIZE_X/g_contexts.size())), 0);
	// upper right
	glTexCoord2f((float)nXRes/(float)g_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X/g_contexts.size()+(i*(GL_WIN_SIZE_X/g_contexts.size())), 0);
	// bottom right
	glTexCoord2f((float)nXRes/(float)g_nTexMapX, (float)nYRes/(float)g_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X/g_contexts.size()+(i*(GL_WIN_SIZE_X/g_contexts.size())), GL_WIN_SIZE_Y/g_contexts.size());
	// bottom left
	glTexCoord2f(0, (float)nYRes/(float)g_nTexMapY);
	glVertex2f((i*(GL_WIN_SIZE_X/g_contexts.size())), GL_WIN_SIZE_Y/g_contexts.size());

	glEnd();
	}

	// Swap the OpenGL display buffers
	glutSwapBuffers();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	for(int i = 0; i < g_contexts.size(); i++)
	{
	switch (key)
	{
	case 27:
		exit (1);
	case '1':
		g_nViewState = DISPLAY_MODE_OVERLAY;
		g_depths[i].GetAlternativeViewPointCap().SetViewPoint(g_images[i]);
		break;
	case '2':
		g_nViewState = DISPLAY_MODE_DEPTH;
		g_depths[i].GetAlternativeViewPointCap().ResetViewPoint();
		break;
	case '3':
		g_nViewState = DISPLAY_MODE_IMAGE;
		g_depths[i].GetAlternativeViewPointCap().ResetViewPoint();
		break;
	case 'm':
		g_contexts[i].SetGlobalMirror(!g_contexts[i].GetGlobalMirror());
		break;
	}
	}
}

int main(int argc, char* argv[])
{
	XnStatus rc;
	EnumerationErrors errors;

	Context b_context;
	rc = b_context.Init();

	NodeInfoList list;
	rc = b_context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, list, &errors);

	int i = 0;
	for (NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it, ++i)
	{
		printf("making node %d\n", i);
		Context l_context;
		l_context.Init();

		//ScriptNode node;
		rc = l_context.Init();

		NodeInfo deviceNodeInfo = *it;
		l_context.CreateProductionTree(deviceNodeInfo);
		//rc = l_context.InitFromXmlFile(SAMPLE_XML_PATH, node, &errors);
		//g_scriptNodes.push_back(node);
		/*if (rc == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return (rc);
		}
		else if (rc != XN_STATUS_OK)
		{
			printf("Open failed: %s\n", xnGetStatusString(rc));
			return (rc);
		}*/

		DepthGenerator l_depth;
		ImageGenerator l_image;

		rc = l_depth.Create(l_context);
		rc = l_image.Create(l_context);

		DepthMetaData l_depthMD;
		ImageMetaData l_imageMD;

		l_depth.GetMetaData(l_depthMD);
		l_image.GetMetaData(l_imageMD);

		rc = l_context.StartGeneratingAll();
		
		g_nTexMapX = (((unsigned short)(l_depthMD.FullXRes()-1) / 512) + 1) * 512;
		g_nTexMapY = (((unsigned short)(l_imageMD.FullYRes()-1) / 512) + 1) * 512;
		g_pTexMaps.push_back((XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel)));
		
		g_images.push_back(l_image);
		g_depths.push_back(l_depth);
		g_contexts.push_back(l_context);
	}



	// OpenGL init
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("OpenNI Simple Viewer");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	// Per frame code is in glutDisplay
	glutMainLoop();


/*	rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return (rc);
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	if (rc != XN_STATUS_OK)
	{
		printf("No depth node exists! Check your XML.");
		return 1;
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	if (rc != XN_STATUS_OK)
	{
		printf("No image node exists! Check your XML.");
		return 1;
	}

	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);

	// Hybrid mode isn't supported in this sample
	if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes())
	{
		printf ("The device depth and image resolution must be equal!\n");
		return 1;
	}

	// RGB is the only image format supported.
	if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		return 1;
	}

	// Texture map init
	g_nTexMapX = (((unsigned short)(g_depthMD.FullXRes()-1) / 512) + 1) * 512;
	g_nTexMapY = (((unsigned short)(g_depthMD.FullYRes()-1) / 512) + 1) * 512;
	g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

	// OpenGL init
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("OpenNI Simple Viewer");
	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	// Per frame code is in glutDisplay
	glutMainLoop();*/

	return 0;
}
