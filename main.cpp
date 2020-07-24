// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.h>
#include <k4a/k4atypes.h>
#include <sys/types.h>
#include <sys/stat.h>
// #include <direct.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/photo/photo.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>

using namespace cv;
using namespace std;

Mat color_image;

class Point3D
{
private:
    /* data */
public:
    Point3D(float, float, float);
    ~Point3D();
    float x,y,z;
    float get_distance(Point3D p);
};

Point3D::Point3D(float a, float b, float c)
{
    x = a;
    y = b;
    z = c;
}

Point3D::~Point3D()
{
}

float Point3D::get_distance(Point3D p){
    return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z));
}

class ClickData
{
private:
    /* data */
public:
    ClickData(/* args */);
    ~ClickData();
    int n_clicks;
    float x1, y1;
    float x,y;
};

ClickData::ClickData(/* args */)
{
    this->n_clicks = 0;
}

ClickData::~ClickData()
{
}


void CallBackFunc(int event, int x, int y, int flags, void* userdataptr)
{
    ClickData* data = (ClickData*) userdataptr;
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        // cout << "Surprisingly, current data->n_clicks is: " << data->n_clicks << endl;
        if (data->n_clicks == 0){
            data->x1 = x;
            data->y1 = y;
        }
        data->n_clicks++;
    }

    if (data->n_clicks > 0){
        data->x = x;
        data->y = y;
    }
    //  else if  ( event == EVENT_RBUTTONDOWN )
    //  {
    //       cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    //  }
    //  else if  ( event == EVENT_MBUTTONDOWN )
    //  {
    //       cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    //  }
    //  else if ( event == EVENT_MOUSEMOVE )
    //  {
    //       cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

    //  }

}



int average_window_filter(const Mat trans_depth_image, int i, int j, int height, int width)
{
    bool found = false;
    int window_size = 1;
    float depth = 0;
    while (found == false)
    {
        window_size *= 2;
        int step = window_size / 2;

        int x_lower = max(i - step, 0);
        int x_upper = min(i + step, height);

        int y_lower = max(j - step, 0);
        int y_upper = min(j + step, width);

        int value = 0;
        int number = 0;

        for (int x = x_lower; x < x_upper; x++)
        {
            for (int y = y_lower; y < y_upper; y++)
            {
                if (trans_depth_image.at<cv::int16_t>(x, y) > 0)
                {
                    found = true;
                    value += trans_depth_image.at<cv::int16_t>(x, y);
                    number += 1;
                }
            }
        }
        if (found == true)
        {
            depth = value / number;
        }
    }
    return depth;
}

static Point3D convert_2d_depth_to_3d_point_cloud(const k4a_calibration_t* calibration, const Mat trans_depth_image, float coordinate_x, float coordinate_y)
{
    int width = calibration->color_camera_calibration.resolution_width;
    printf("width = %d ", width);
    int height = calibration->color_camera_calibration.resolution_height;
    printf("height = %d ", height);

    int valid;
    float depth;
    int coordinate_x_int = static_cast<int>(coordinate_x);
    int coordinate_y_int = static_cast<int>(coordinate_y);
    depth = average_window_filter(trans_depth_image, coordinate_x_int, coordinate_y_int, height, width);
    // depth = (trans_depth_image.at<cv::int16_t>(coordinate_x_int,coordinate_y_int));


    k4a_float3_t ray;
    k4a_float2_t point_2d;

    point_2d.xy.x = coordinate_x;
    point_2d.xy.y = coordinate_y;

    if (K4A_RESULT_SUCCEEDED == k4a_calibration_2d_to_3d(calibration, &point_2d, depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid))
    {
        cout << "x = " << ray.xyz.x << " | y = " << ray.xyz.y << " | z = " << ray.xyz.z << " | depth = " << depth << " | valid = " << valid << endl;
        return Point3D(ray.xyz.x, ray.xyz.y, ray.xyz.z);
    }
    else 
    {
        cout << "k4a_calibration_2d_to_3d failed for the current input pixel!" << endl;
    }
}


int main()
{

    // Start by counting the number of connected devices
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }
    else
    {
        printf("Found %d connected devices:\n", device_count);
    }

    // Define the Exit block
    int returnCode = 1;

    // Initialize the the device and capture attributes
    k4a_device_t device = NULL;
    k4a_capture_t capture = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;

    // Initialize the frame count
    int totalFrame = 1;
    int captureFrameCount = totalFrame;
    printf("Capturing %d frames\n", captureFrameCount);

    // Set the configuration of device, you can also set it after open the device but before starting the camera
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;  // <==== For Color image
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P; //K4A_COLOR_RESOLUTION_2160P; 720, 1080, 
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;  // <==== For Depth image
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.synchronized_images_only = true;

    // Open the device
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
        goto Exit;
    }

    // Set the calibration
    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    // Start the camera
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start device\n");
        goto Exit;
    }

    // Start to receive the captures and framess
    while (captureFrameCount-- > 0)
    {
        // Get a depth frame
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            goto Exit;
        }

        // Probe for a depth16 image
        const k4a_image_t image_depth = k4a_capture_get_depth_image(capture);
        if (image_depth != NULL)
        {
            // Get the sizes of depth image
            int width = k4a_image_get_width_pixels(image_depth);
            int height = k4a_image_get_height_pixels(image_depth);
            int strides = k4a_image_get_stride_bytes(image_depth);
            printf("Depth image height, width and strides: %d, %d, %d\n", height, width, strides);

            // Store the image using opencv Mat
            uint16_t* depth_image_data = (uint16_t*)(void*)k4a_image_get_buffer(image_depth);
            const Mat depth_image(height, width, CV_16U, (void*)depth_image_data, Mat::AUTO_STEP);

            // Display the images
            namedWindow("foobar", WINDOW_AUTOSIZE);
            imshow("foobar", depth_image);
            waitKey(1000);
        }
        else
        {
            printf(" | Depth16 None\n");
        }

        // Probe for a color image
        k4a_image_t image_color = k4a_capture_get_color_image(capture);
        if (image_color != NULL)
        {
            // Get the sizes of color image
            int width = k4a_image_get_width_pixels(image_color);
            int height = k4a_image_get_height_pixels(image_color);
            int strides = k4a_image_get_stride_bytes(image_color);
            printf("Color image height, width and strides: %d, %d, %d\n", height, width, strides);

            // Store the image using opencv Mat
            uint8_t* color_image_data = k4a_image_get_buffer(image_color);
            color_image = Mat(height, width, CV_8UC4, (void*)color_image_data, Mat::AUTO_STEP);

            // Display the images
            namedWindow("foobar", WINDOW_AUTOSIZE);
            imshow("foobar", color_image);
            waitKey(1000);
        }
        else
        {
            printf(" | Color None                       ");
        }

        
        // 9.2  derive the depth value in the color camera geometry using the function k4a_transformation_depth_image_to_color_camera().
        k4a_transformation_t transformation = NULL;
        k4a_image_t transformed_depth_image = NULL;
        int width = k4a_image_get_width_pixels(image_color);
        int height = k4a_image_get_height_pixels(image_color);

        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            width,
            height,
            width * (int)sizeof(uint16_t),
            &transformed_depth_image))
        {
            printf("Failed to create transformed color image\n");
            return false;
        }
        else
        {

            // Transform the depth image to the size of color camera
            transformation = k4a_transformation_create(&calibration);
            k4a_transformation_depth_image_to_color_camera(transformation, image_depth, transformed_depth_image);
        
            // Store the image using opencv Mat
            uint16_t* transformed_depth_image_data = (uint16_t*)(void*)k4a_image_get_buffer(transformed_depth_image);
            const Mat trans_depth_image(height, width, CV_16U, (void*)transformed_depth_image_data, Mat::AUTO_STEP);
            Mat valid_color_img = color_image.clone();
            // cout << "The height and width are : " << height << width <<endl;
            // // exit(0);
            // cout << "Matrix type " << valid_color_img.type();
            // cout << " Matrix Size " << valid_color_img.rows << "x" << valid_color_img.cols << endl;
            // for (int i=0; i<height; i++){
            //     for (int j=0; j<width; j++){
            //         // valid_color_img.at<Vec3b>(Point(j, i))[0] = 100;
            //         // valid_color_img.at<Vec3b>(Point(j, i))[1] = 100;
            //         // valid_color_img.at<Vec3b>(Point(j, i))[2] = 100;
            //         // cout << "Here " <<  i << " " << j <<endl;
            //         if (trans_depth_image.at<cv::int16_t>(i, j) == 0){
            //             // cout <<" The depth pixel value at " <<i <<" and " << j << " is " << val << endl;
            //             // cout << "Replacing pixel" << endl;
            //             valid_color_img.at<Vec3b>(i, j)[0] = 0;
            //             valid_color_img.at<Vec3b>(i, j)[1] = 0;
            //             valid_color_img.at<Vec3b>(i, j)[2] = 0;
            //         }else{
            //             valid_color_img.at<Vec3b>(i, j)[0] = 100;
            //             valid_color_img.at<Vec3b>(i, j)[1] = 100;
            //             valid_color_img.at<Vec3b>(i, j)[2] = 100;
            //             // valid_color_img.at<Vec3b>(Point(j, i))[1] = color_image.at<Vec3b>(Point(j, i))[1];
            //             // valid_color_img.at<Vec3b>(Point(j, i))[2] = color_image.at<Vec3b>(Point(j, i))[2];
            //         }
            //     }
            // }
            // Display the transformed depth images
            namedWindow("foobar", WINDOW_AUTOSIZE);
            imshow("foobar", trans_depth_image);
            waitKey(1000);

            // Find the point xy coordinate from color image, this pair of points belongs to the short edge of the shelf
            // float point1_row = (height / 2) * 3.7 / 8.9;
            // float point1_column = (width / 2) * 6.3 / 15.95;
            // convert_2d_depth_to_3d_point_cloud(&calibration, trans_depth_image, point1_row, point1_column);

            // float point2_row = (height / 2) * 3.8 / 8.9;
            // float point2_column = (width / 2) * 8.6 / 15.95;
            // convert_2d_depth_to_3d_point_cloud(&calibration, trans_depth_image, point2_row, point2_column);

            float point1_row = 0;
            float point1_column = 0;
            float point2_row = 0;
            float point2_column = 0;

            namedWindow("foobar", WINDOW_AUTOSIZE);
            // Mat measure_img = color_image.clone();
            ClickData* click_data = new ClickData();
            //set the callback function for any mouse event
            setMouseCallback("foobar", CallBackFunc, click_data);
            uint pressed_char;
            do{
                Mat measure_img = valid_color_img.clone();
                if (click_data->n_clicks>0){
                    line (measure_img, Point(click_data->x1, click_data->y1), Point(click_data->x, click_data->y), Scalar(0,0,0), LINE_4);

                    Point3D p1 = convert_2d_depth_to_3d_point_cloud(&calibration, trans_depth_image, click_data->x1, click_data->y1);
                    Point3D p2 = convert_2d_depth_to_3d_point_cloud(&calibration, trans_depth_image, click_data->x, click_data->y);

                    cout<<"The distance is "<< p1.get_distance(p2) << endl;

                }
                imshow("foobar", measure_img);
                pressed_char = waitKey(10);
                if (pressed_char == 32){
                    click_data->n_clicks = 0;
                }
            }while (pressed_char!=27);

            // float point3_row = (height / 2) * 4.8 / 8.9;
            // float point3_column = (width / 2) * 10.6 / 15.95;
            // convert_2d_depth_to_3d_point_cloud(&calibration, trans_depth_image, point3_row, point3_column);

            // float point4_row = (height / 2) * 5.1 / 8.9;
            // float point4_column = (width / 2) * 24.85 / 15.95;
            // convert_2d_depth_to_3d_point_cloud(&calibration, trans_depth_image, point4_row, point4_column);
        }

        // release images
        k4a_image_release(image_depth);
        k4a_image_release(image_color);
        k4a_image_release(transformed_depth_image);

        // release capture
        k4a_capture_release(capture);

    }

    returnCode = 0;
Exit:
    if (device != NULL)
    {
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
    }

    return returnCode;
}
