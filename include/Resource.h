#ifndef RESOURCE_H_
#define RESOURCE_H_

#include <iostream>
#include <numeric>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include <fstream>
#include <pthread.h>

//******** include NVX, these head files are installed with visionworks********
#include <NVX/nvx.h>
#include <NVX/nvx_timer.hpp>
#include <NVX/nvx_opencv_interop.hpp>
#include <NVX/nvx_api_macros.h>
#include <NVX/nvx_compatibility.h>
#include <NVX/nvx_cuda_interop.h>
#include <NVX/nvxcu.h>
#include <NVX/nvx_export.h>
#include <NVX/nvx_version.h>

//******** include VX, these head files are installed with visionworks**********
#include <VX/vx_api.h>
#include <VX/vx.h>
#include <VX/vx_nodes.h>
#include <VX/vxu.h>
#include <VX/vx_compatibility.h>
#include <VX/vx_kernels.h>
#include <VX/vx_types.h>
#include <VX/vx_vendors.h>

//********include Eigen head file***********************************************
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/src/Core/MatrixBase.h>

//******** OpenCV includes*******************************************************
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/types.hpp>
//#include <opencv2/core/eigen.hpp>


//******** include mavlink head file*********************************************
#include <position_controller.h>


//******** Define parameters and function*********************************************
#define PORT "/dev/ttyUSB0"
#define BAUD 921600
#define DEFAULT_WAITKEY_DELAY  1
#define DETECTION_FREQUENCY 1
#define AVG_FREQUENCY 10
#define STEP 2

#define ERROR_CHECK_STATUS( status ) { \
        vx_status status_ = (status); \
        if(status_ != VX_SUCCESS) { \
            printf("ERROR: failed with status = (%d) at " __FILE__ "#%d\n", status_, __LINE__); \
            exit(1); \
        } \
    }
#define ERROR_CHECK_OBJECT( obj ) { \
        vx_status status_ = vxGetStatus((vx_reference)(obj)); \
        if(status_ != VX_SUCCESS) { \
            printf("ERROR: failed with status = (%d) at " __FILE__ "#%d\n", status_, __LINE__); \
            exit(1); \
        } \
    }

// Namespaces
using namespace cv;
using namespace cv::ml;
using namespace std;
using namespace std::chrono;
using namespace Eigen;
//using namespace cv::cuda;

struct graphEdgeParams
{
    vx_int32    CannyLowerThresh;
    vx_int32    CannyUpperThresh;

    graphEdgeParams()
        : CannyLowerThresh(230),
          CannyUpperThresh(250)
     {}
};

struct graphHarrisParams
{
    vx_int32    Thresh;

    graphHarrisParams():Thresh(130) //125
    {}
};

struct graphTrackParams
{
    int avg_fps;
    int avg_sumfps;
    int V_count;
    int track_count;
    int img_region_x1;
    int img_region_y1;
    int img_region_x2;
    int img_region_y2;
    double total_time;

    graphTrackParams()
    :avg_fps(0),
     avg_sumfps(0),
     V_count(0),
     track_count(0),
     img_region_x1(0),
     img_region_y1(0),
     img_region_x2(648),
     img_region_y2(488),
     total_time(0)
     {}

};

void VX_CALLBACK log_callback( vx_context    context,
                               vx_reference  ref,
                               vx_status     status,
                               const vx_char string[] )
{
    printf( "LOG: [ status = %d ] %s\n", status, string );
    fflush( stdout );
}

bool eigenvalue_check (MatrixXf new_points);

bool AbortRequested();

vx_df_image convertMatTypeToImageFormat(int mat_type);

vx_status createMatFromImage(cv::Mat &mat, vx_image image);

void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles);

void getPose(vector<cv::Point3d> target_points, vector<cv::Point2f> image_points, cv::Mat k, cv::Mat d, cv::Mat& r_vec, cv::Mat& t_vec);

#endif /* RESOURCE_H_ */
