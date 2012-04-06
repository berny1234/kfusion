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
#include <XnPropNames.h>

#include "kfusion.h"
#include "helpers.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace xn;

#include "perfstats.h"

using namespace std;
using namespace TooN;

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

#ifndef GL_SGIS_generate_mipmap
#define GL_GENERATE_MIPMAP_SGIS           0x8191
#define GL_GENERATE_MIPMAP_HINT_SGIS      0x8192
#endif


//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
XnMapOutputMode g_defaultMode;

std::vector<Context> g_contexts;
std::vector<DepthGenerator> g_depths;
std::vector<ImageGenerator> g_images;

KFusion kfusion;
Image<uchar4, HostDevice> lightScene, lightModel;

std::vector<Image<uchar4, HostDevice>> depths;
//Image<uint16_t, HostDevice> depthImage;

std::vector<Image<uint16_t, HostDevice>> depthImages;

const float3 light = make_float3(1.0, -2, 1.0);
const float3 ambient = make_float3(0.1, 0.1, 0.1);

SE3<float> initPose;
SE3<float> initPose2;

int counter = 0;
bool reset = true;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------


void CloseKinect(){
	for(int i =0; i < g_contexts.size(); i++)
	{
		g_contexts[i].Shutdown();
	}
}

void display(void){
	const uint2 imageSize = kfusion.configuration.inputSize;
	static bool integrate = true;

	glClear( GL_COLOR_BUFFER_BIT );
	const double startFrame = Stats.start();
	const double startProcessing = Stats.sample("kinect");
	////////////////////////////////////////////////////////////////////////
	//DepthFrameKinect();
	for(int i = 0; i < g_contexts.size(); i++)
	{
		kfusion.setKFusionDevice(i);
		XnStatus rc = XN_STATUS_OK;
		rc = g_contexts[i].WaitAnyUpdateAll();
		if (rc != XN_STATUS_OK)
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return;
		}

		xnOSMemCopy(depthImages[i].data(), g_depths[i].GetDepthMap(), depthImages[i].size.x*depthImages[i].size.y*sizeof(XnDepthPixel));

		////////////////////////////////////////////////////////////////////////

		kfusion.setKinectDeviceDepth(depthImages[i].getDeviceImage());
		Stats.sample("raw to cooked");

		integrate = kfusion.Track();
		Stats.sample("track");

		if(integrate || reset){
			kfusion.Integrate();
			Stats.sample("integrate");
			reset = false;
		}
		cudaDeviceSynchronize();
	}

	//TODO: this is now just debug output for the first two devices. 
	renderLight( lightModel.getDeviceImage(), kfusion.vertex, kfusion.normal, light, ambient);
	renderLight( lightScene.getDeviceImage(), kfusion.inputVertex[0], kfusion.inputNormal[0], light, ambient );
	renderTrackResult( depths[0].getDeviceImage(), kfusion.reductions[0] );
	renderTrackResult( depths[1].getDeviceImage(), kfusion.reductions[1] );
	cudaDeviceSynchronize();

	Stats.sample("render");

	glClear(GL_COLOR_BUFFER_BIT);
	glRasterPos2i(0,imageSize.y * 0);
	glDrawPixels(lightScene);
	glRasterPos2i(imageSize.x, imageSize.y * 0);
	glDrawPixels(depths[0]);
	glRasterPos2i(0,imageSize.y * 1);
	glDrawPixels(lightModel);
	glRasterPos2i(imageSize.x,imageSize.y * 1);
	glDrawPixels(depths[1]);
	const double endProcessing = Stats.sample("draw");

	Stats.sample("total", endProcessing - startFrame, PerfStats::TIME);
	Stats.sample("total_proc", endProcessing - startProcessing, PerfStats::TIME);

	if(printCUDAError())
		exit(1);

	++counter;

	if(counter % 50 == 0){
		Stats.print();
		Stats.reset();
		cout << endl;
	}


	glutSwapBuffers();
}

void idle(void){
	glutPostRedisplay();
}


void keys(unsigned char key, int x, int y){
	switch(key){
	case 'c':
		kfusion.Reset();
		for(int i = 0; i<g_contexts.size(); i++)
		{
			kfusion.setPose(toMatrix4(initPose), i);
		}
		reset = true;
		break;
	case 'q':
		exit(0);
		break;
	}
}

void reshape(int width, int height){
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glColor3f(1.0f,1.0f,1.0f);
	glRasterPos2f(-1, 1);
	glOrtho(-0.375, width-0.375, height-0.375, -0.375, -1 , 1); //offsets to make (0,0) the top left pixel (rather than off the display)
	glPixelZoom(1,-1);
}

void exitFunc(void){
	CloseKinect();
	kfusion.Clear();
	cudaDeviceReset();
}


int main(int argc, char* argv[])
{
	const float size = (argc > 1) ? atof(argv[1]) : 2.f;
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

		DepthGenerator l_depth;
		ImageGenerator l_image;

		rc = l_depth.Create(l_context);
		rc = l_image.Create(l_context);

		DepthMetaData l_depthMD;
		ImageMetaData l_imageMD;

		l_depth.GetMetaData(l_depthMD);
		l_image.GetMetaData(l_imageMD);

		rc = l_context.StartGeneratingAll();

		g_defaultMode.nXRes = 640;
		g_defaultMode.nYRes = 480;
		g_defaultMode.nFPS = 30;
		rc = l_depth.SetMapOutputMode(g_defaultMode);
		if (rc != XN_STATUS_OK)
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return -1;
		}

		l_depth.SetIntProperty("OutputFormat", 0);
		l_depth.SetIntProperty("HoleFilter", TRUE);
		printf("map props: \n bytes: %d\n maxdepth: %d \n ",l_depth.GetBytesPerPixel(),l_depth.GetDeviceMaxDepth());

		g_images.push_back(l_image);
		g_depths.push_back(l_depth);
		g_contexts.push_back(l_context);
	}


	KFusionConfig config;

	// it is enough now to set the volume resolution once.
	// everything else is derived from that.
	// config.volumeSize = make_uint3(64);
	config.volumeSize = make_uint3(512);
	// config.volumeSize = make_uint3(256);

	// these are physical dimensions in meters
	config.volumeDimensions = make_float3(size);
	config.nearPlane = 0.4f;
	config.farPlane = 5.0f;
	config.mu = 0.1;
	config.combinedTrackAndReduce = false;
	config.numUsedDevices = g_contexts.size();

	// change the following parameters for using 640 x 480 input images
	config.inputSize = make_uint2(640,480); 
	config.camera =  make_float4(297.12732*2, 296.24240*2, 169.89365*2, 121.25151*2);

	// config.iterations is a vector<int>, the length determines
	// the number of levels to be used in tracking
	// push back more then 3 iteraton numbers to get more levels.
	config.iterations[0] = 10;
	config.iterations[1] = 5;
	config.iterations[2] = 4;

	config.dist_threshold = (argc > 2 ) ? atof(argv[2]) : config.dist_threshold;
	config.normal_threshold = (argc > 3 ) ? atof(argv[3]) : config.normal_threshold;

	initPose = SE3<float>(makeVector(size/2, size/2, 0, 0, 0, 0));

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE );
	glutInitWindowSize(config.inputSize.x * 2, config.inputSize.y * 2);
	glutCreateWindow("kfusion");

	kfusion.Init(config);
	if(printCUDAError())
		exit(1);

	initPose = SE3<float>(makeVector(size/2, size/2, 0, 0, 0, 0));
	kfusion.addPose(toMatrix4(initPose));
	initPose2 = SE3<float>(makeVector(size/2, -size/2, 0, 0, 0, 0));
	kfusion.addPose(toMatrix4(initPose));


	lightScene.alloc(config.inputSize), lightModel.alloc(config.inputSize);
//	depthImage.alloc(make_uint2(640, 480));
	depthImages.resize(g_contexts.size());
	depths.resize(g_contexts.size());
	for(int i = 0; i<g_contexts.size(); i++)
	{
		depthImages[i].alloc(config.inputSize);
		depths[i].alloc(config.inputSize);
	}

	atexit(exitFunc);
	glutDisplayFunc(display);
	glutKeyboardFunc(keys);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);

	glutMainLoop();

	return 0;
}
