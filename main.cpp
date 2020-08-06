////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
 
#include "tcamimage.h"

#include "opencv2/opencv.hpp"
#include "Resource.h"

using namespace gsttcam;

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

///******** Declaration *******************************************************************************

//    graphEdgeParams EdgeParams;    
    graphHarrisParams harrisparams;
    graphTrackParams trackparams;
    nvx::Timer timer1, timer2, timer3, timer4, timer5, timer6, timer7, timer8, timer9, timer10, timer11, timer12;

///******** Mavlink port setting***********************************************************************

    Multithreaded_Interface mti;
    //mti.start(PORT, BAUD);

///******** Setpoint parameters***********************************************************************

    Position_Controller pc(&mti);

    float cam_x = 0.0;
    float cam_y = 0.0;
    float cam_z = 0.0;
    float cam_yaw = 0.0;

    float cam_x_pre = 0.0;
    float cam_y_pre = 0.0;
    float cam_z_pre = 0.0;

    float cam_x_sm = 0.0;
    float cam_y_sm = 0.0;
    float cam_z_sm = 0.0;

    float tracking_x_desired = -0.6;
    float tracking_y_desired = 0.10;
    float tracking_z_desired = 0.10;
    float tracking_yaw_desired = 0.0;

///******** Camera matrix ***********************************************************************

    Mat K = (cv::Mat_<double>(3, 3) << 228.4206, 0, 328.2429, 0, 229.3223, 240.1774, 0, 0, 1);
    Mat inv_K = (cv::Mat_<double>(3,3) << 0.0044, 0, -1.4370, 0, 0.0044, -1.0473, 0, 0, 1);
    double inv_K0 = inv_K.at<double>(0);
    double inv_K2 = inv_K.at<double>(2);
    double inv_K4 = inv_K.at<double>(4);
    double inv_K5 = inv_K.at<double>(5);
    Mat dist_coeff = (cv::Mat_<double>(5, 1) << -0.0079, 0.0088, -0.0015, 0, 0);
    Mat rvec, tvec, rvec2, tvec2, R, Rt, p;

///******** Target parameters ***********************************************************************

    double obj_w = 0.21;//1.109;//0.21;
    double obj_h = 0.15;//0.771;//0.15;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

///******* Store 3D features **********************************************************************

    vector<cv::Point3d> obj_points, img_points;
    vector<cv::Point2f> new_img_points, img_points_lu, img_points_ru, img_points_lb, img_points_rb; 
    for (int h = 0; h < 2; h++)
        for (int w = 0; w < 2; w++)
            obj_points.push_back(cv::Point3d(obj_w * w, obj_h * h, 0));

    /*obj_points.push_back(cv::Point3d(0, 0, 0.006));
    obj_points.push_back(cv::Point3d(0.27, 0, 0));
    obj_points.push_back(cv::Point3d(0, 0.19, 0));
    obj_points.push_back(cv::Point3d(0.27, 0.19, 0.006));*/


///******* Noise reduction ************************************************************************

    vector<cv::Point2f> lu_collect;
    vector<cv::Point2f> ru_collect;
    vector<cv::Point2f> lb_collect;    
    vector<cv::Point2f> rb_collect;
    
    Point2f lu_result, ru_result, lb_result, rb_result;

    vector<double> data_collect_x;
    vector<double> data_collect_y;
    vector<double> data_collect_z;

    vector<double> fir_b{0.0027,0.0029,0.0032,0.0039,0.0048,0.0059,0.0072,0.0088,0.0105,0.0124,
                         0.0144,0.0165,0.0187,0.0209,0.0230,0.0252,0.0273,0.0292,0.0310,0.0327,
                         0.0341,0.0353,0.0363,0.0370,0.0374,0.0375,0.0374,0.0370,0.0363,0.0353,
                         0.0341,0.0327,0.0310,0.0292,0.0273,0.0252,0.0230,0.0209,0.0187,0.0165,
                         0.0144,0.0124,0.0105,0.0088,0.0072,0.0059,0.0048,0.0039,0.0032,0.0029};   

    int Order = 50;
    

///********Camera parameter setting*********************************************
    gst_init(&argc, &argv);

    cv::Mat OpenCVImage;
    cv::Mat display;
    // Initialize our TcamCamera object "cam" with the serial number
    TcamImage cam("46910274");
    int image_W = 720;
    int image_H = 540;
    
    // Set a color video format, resolution and frame rate
    cam.set_capture_format("BGRx", FrameSize{image_W,image_H}, FrameRate{480,1}); //(1) USB3.1 Gen2 cable is needed.  //video format: GRAY8, BGRx

    // Start the camera
    cam.start();

    sleep(1);
    printf("Start Snap\n");

///******** Set the OpenVX parameters*******************************************************************

    vx_uint32  width                   = image_W;               // image width
    vx_uint32  height                  = image_H;               // image height
    vx_size    max_keypoint_count      = 10000;                 // maximum number of keypoints to track                   default:10000
    vx_float32 harris_strength_thresh  = 0.01;//0.01;                  // minimum corner strength to keep a corner               default:0.0005f    Tuned:0.01f
    vx_float32 harris_min_distance     = 5.0f;                  // radial L2 distance for non-max suppression             default:5.0f
    vx_float32 harris_sensitivity      = 0.04f;                 // multiplier k in det(A) - k * trace(A)^2                default:0.04f      Tuned:0.15f
    vx_int32   harris_gradient_size    = 3;//7;                     // window size for gradient computation                   default:3          Tuned:5
    vx_int32   harris_block_size       = 3;//7;                     // block window size for Harris corner score              default:3          Tuned:7
    vx_uint32  lk_pyramid_levels       = 6;                     // number of pyramid levels for optical flow              default:6          Tuned:3
    vx_float32 lk_pyramid_scale        = VX_SCALE_PYRAMID_HALF; // pyramid levels scale by factor of two
    vx_enum    lk_termination          = VX_TERM_CRITERIA_BOTH; // iteration termination criteria (eps & iterations)
    vx_float32 lk_epsilon              = 0.01f;                 // convergence criterion
    vx_uint32  lk_num_iterations       = 5;                     // maximum number of iterations                           default:5
    vx_bool    lk_use_initial_estimate = vx_false_e;            // don't use initial estimate
    vx_uint32  lk_window_dimension     = 30;                     // window size for evaluation                             default:6
    vx_float32 trackable_kp_ratio_thr  = 0.8f;                  // threshold for the ration of tracked keypoints to all   default:0.8


///******** Create context, the OpenVX object domain*****************************************************

    vx_context context = vxCreateContext();
    ERROR_CHECK_OBJECT( context );

    vxRegisterLogCallback( context, log_callback, vx_false_e );

///******** Create OpenVX objects************************************************************************
    // Image container
    vx_image input_U8_image = vxCreateImage( context, width, height, VX_DF_IMAGE_U8);
    ERROR_CHECK_OBJECT( input_U8_image );

    vx_image input_RGB_image = vxCreateImage( context, width, height, VX_DF_IMAGE_RGB);
    ERROR_CHECK_OBJECT( input_RGB_image );

    vx_image image_threshold = vxCreateImage( context, width, height, VX_DF_IMAGE_U8);
    ERROR_CHECK_OBJECT( image_threshold );

    vx_image image_edges = vxCreateImage(context, width, height, VX_DF_IMAGE_U8);
    ERROR_CHECK_OBJECT( image_edges );

    vx_pyramid pyramidExemplar = vxCreatePyramid( context, lk_pyramid_levels,lk_pyramid_scale, width, height, VX_DF_IMAGE_U8 );
    ERROR_CHECK_OBJECT( pyramidExemplar );

    //Image and keypoints delay container
    vx_delay pyramidDelay   = vxCreateDelay( context, ( vx_reference )pyramidExemplar, STEP);
    ERROR_CHECK_OBJECT( pyramidDelay );
    ERROR_CHECK_STATUS( vxReleasePyramid( &pyramidExemplar ) );

    vx_array keypointsExemplar = vxCreateArray( context, VX_TYPE_KEYPOINT, max_keypoint_count );
    ERROR_CHECK_OBJECT( keypointsExemplar );

    vx_delay keypointsDelay = vxCreateDelay( context, ( vx_reference )keypointsExemplar, STEP);
    ERROR_CHECK_STATUS( vxReleaseArray( &keypointsExemplar ) );

    vx_pyramid currentPyramid  = ( vx_pyramid ) vxGetReferenceFromDelay( pyramidDelay, 0 );
    vx_pyramid previousPyramid = ( vx_pyramid ) vxGetReferenceFromDelay( pyramidDelay, -1);
    vx_array currentKeypoints  = ( vx_array )   vxGetReferenceFromDelay( keypointsDelay, 0 );
    vx_array previousKeypoints = ( vx_array )   vxGetReferenceFromDelay( keypointsDelay, -1 );

    ERROR_CHECK_OBJECT( currentPyramid );
    ERROR_CHECK_OBJECT( previousPyramid );
    ERROR_CHECK_OBJECT( currentKeypoints );
    ERROR_CHECK_OBJECT( previousKeypoints );

    // Graph declare
    vx_graph graphHarris = vxCreateGraph( context );
    vx_graph graphTrack  = vxCreateGraph( context );
    vx_graph graphEdges = vxCreateGraph ( context );
    ERROR_CHECK_OBJECT( graphHarris );
    ERROR_CHECK_OBJECT( graphTrack );
    ERROR_CHECK_OBJECT( graphEdges );

    vx_scalar strength_thresh      = vxCreateScalar( context, VX_TYPE_FLOAT32, &harris_strength_thresh );
    vx_scalar min_distance         = vxCreateScalar( context, VX_TYPE_FLOAT32, &harris_min_distance );
    vx_scalar sensitivity          = vxCreateScalar( context, VX_TYPE_FLOAT32, &harris_sensitivity );
    vx_scalar epsilon              = vxCreateScalar( context, VX_TYPE_FLOAT32, &lk_epsilon );
    vx_scalar num_iterations       = vxCreateScalar( context, VX_TYPE_UINT32,  &lk_num_iterations );
    vx_scalar use_initial_estimate = vxCreateScalar( context, VX_TYPE_BOOL,    &lk_use_initial_estimate );
    ERROR_CHECK_OBJECT( strength_thresh );
    ERROR_CHECK_OBJECT( min_distance );
    ERROR_CHECK_OBJECT( sensitivity );
    ERROR_CHECK_OBJECT( epsilon );
    ERROR_CHECK_OBJECT( num_iterations );
    ERROR_CHECK_OBJECT( use_initial_estimate );

    vx_image virt_blurred = vxCreateVirtualImage(graphEdges, width, height, VX_DF_IMAGE_U8);
    vx_image virt_equalized = vxCreateVirtualImage(graphEdges, width, height, VX_DF_IMAGE_U8);
    vx_image virt_dilate = vxCreateVirtualImage(graphHarris, width, height, VX_DF_IMAGE_U8);

    // Threshold value
    vx_threshold thresh = vxCreateThreshold(context, VX_THRESHOLD_TYPE_BINARY, VX_TYPE_UINT8);
    ERROR_CHECK_OBJECT(thresh);
    ERROR_CHECK_STATUS( vxSetThresholdAttribute(thresh, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_VALUE, &harrisparams.Thresh, sizeof(harrisparams.Thresh)));

    // Create OpenVX Threshold to hold Canny thresholds
/*    vx_threshold CannyThreshold = vxCreateThreshold(context, VX_THRESHOLD_TYPE_RANGE, VX_TYPE_INT32);
    ERROR_CHECK_OBJECT(CannyThreshold);
    ERROR_CHECK_STATUS( vxSetThresholdAttribute(CannyThreshold, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_LOWER,
                                             &EdgeParams.CannyLowerThresh, sizeof(EdgeParams.CannyLowerThresh)) );
    ERROR_CHECK_STATUS( vxSetThresholdAttribute(CannyThreshold, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_UPPER,
                                             &EdgeParams.CannyUpperThresh, sizeof(EdgeParams.CannyUpperThresh)) );*/

///******** Create OpenVX nodes*********************************************************************************
    // Edge    
    /*vx_node nodesEdge[] =
    {
        vxMedian3x3Node(graphEdges, input_U8_image, virt_blurred),

        vxEqualizeHistNode(graphEdges, virt_blurred, virt_equalized),

    	vxCannyEdgeDetectorNode(graphEdges, virt_equalized, CannyThreshold, 3, VX_NORM_L1, image_edges)
    };
    for( vx_size i = 0; i < sizeof( nodesEdge ) / sizeof( nodesEdge[0] ); i++ )
    {
        ERROR_CHECK_OBJECT( nodesEdge[i] );
        ERROR_CHECK_STATUS( vxReleaseNode( &nodesEdge[i] ) );
    }
    ERROR_CHECK_STATUS (vxVerifyGraph ( graphEdges));*/


    // Detect
    vx_node nodesHarris[] =
    {
    	vxThresholdNode( graphHarris, input_U8_image, thresh, image_threshold),
	//vxDilate3x3Node( graphHarris, image_threshold, virt_dilate),
        //vxCannyEdgeDetectorNode(graphHarris, image_threshold, CannyThreshold, 3, VX_NORM_L1, image_edges),
        //vxGaussianPyramidNode( graphHarris, image_threshold , currentPyramid ),
    	vxHarrisCornersNode( graphHarris, image_threshold , strength_thresh, min_distance, sensitivity, harris_gradient_size, harris_block_size, currentKeypoints, NULL )
    };
    for( vx_size i = 0; i < sizeof( nodesHarris ) / sizeof( nodesHarris[0] ); i++ )
    {
        ERROR_CHECK_OBJECT( nodesHarris[i] );
        ERROR_CHECK_STATUS( vxReleaseNode( &nodesHarris[i] ) );
    }
    ERROR_CHECK_STATUS( vxVerifyGraph( graphHarris ) );

    // Track
    vx_node nodesTrack[] =
    {
    	vxThresholdNode( graphTrack, input_U8_image, thresh, image_threshold),
        //vxCannyEdgeDetectorNode(graphTrack, image_threshold, CannyThreshold, 3, VX_NORM_L1, image_edges),
    	vxGaussianPyramidNode( graphTrack,  image_threshold  , currentPyramid ),
    	vxOpticalFlowPyrLKNode( graphTrack, previousPyramid, currentPyramid,
    		                                previousKeypoints, currentKeypoints, currentKeypoints, //previousKeypoints
    		                                lk_termination, epsilon, num_iterations,
    		                                use_initial_estimate, lk_window_dimension )
    };
    for( vx_size i = 0; i < sizeof( nodesTrack ) / sizeof( nodesTrack[0] ); i++ )
    {
        ERROR_CHECK_OBJECT( nodesTrack[i] );
        ERROR_CHECK_STATUS( vxReleaseNode( &nodesTrack[i] ) );
    }
    ERROR_CHECK_STATUS( vxVerifyGraph( graphTrack ) );

///******** Execute loop *****************************************************************************************

    //for( int frame_index = 0; !AbortRequested(); frame_index++ )
    int frame_index = 0;

    while(1)
    {
        // Snap an Image with X ms timeout. Should be set accordingly to the frame rate.
        if( cam.snapImage(4) )
        {
	    OpenCVImage.create( cam.getHeight(),cam.getWidth(),CV_8UC(cam.getBytesPerPixel()));  
            memcpy( OpenCVImage.data, cam.getImageData(), cam.getImageDataSize());
            display = OpenCVImage.clone();

	    cv::imshow("Raw image",display);

	    if (char(cvWaitKey(2))=='q') break;
	}
        else
        {
            printf("Timeout at snapImage()\n");
        }
    }

    printf("Press enter key to end program.");
    // Simple implementation of "getch()", wait for enter key.
    char dummyvalue[10];
    scanf("%c",dummyvalue);

    cam.stop();

    return 0;
}
