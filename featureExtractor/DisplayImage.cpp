#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>

// #include <cuda.h>

using namespace cv;

int main(int argc, char** argv )
{

    // int driver_version = 0, runtime_version = 0;

    // cuDriverGetVersion(&driver_version);
    // cudaRuntimeGetVersion(&runtime_version);

    // printf("Driver Version: %d\n"
    //        "Runtime Version: %d\n",
    //        driver_version, runtime_version);

    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    Mat image;
    image = imread( argv[1], 1 );
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);
    waitKey(0);
    
    return 0;
}