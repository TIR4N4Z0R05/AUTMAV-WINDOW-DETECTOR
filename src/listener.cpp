
// %Tag(FULLTEXT)%

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 using namespace cv;
 using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
   {
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
     
   public:
     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, 
    
         &ImageConverter::imageCb_d, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
       cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
  /*   void imageCb(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
   
       // Draw an example circle on the video stream
       if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
         cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
   
       // Update GUI Window
       cv::imshow(OPENCV_WINDOW, cv_ptr->image);
       cv::waitKey(3);
       
       // Output modified video stream
       image_pub_.publish(cv_ptr->toImageMsg());
     }*/
     
      void imageCb_d(const sensor_msgs::ImageConstPtr& msg)
  {
/*
	double source_po_x=0;
	double source_po_y=0;
	double source_po_z=0;
	double source_po=0;
	*/
    cv_bridge::CvImagePtr cv_ptr;
	
	//printf ("Decimals: %d %d %d\n", msg->height , msg->width ,msg->step);
/*	
	double fovh=58.0 / 57.3  ;
	double fovv=45.0 / 57.3 ;
	*/
	
	//for (uint32_t row=0 ; row <= msg->height ; ++row)
	
	//{
		//for (uint32_t step=0 ; step <= msg->step ; ++step)
		//{
			//source_po=100.0/((msg->data[step*row]+1.0) *( msg->data[step*row] +1.0)* msg->step*msg->height);
			
			//source_po_x=source_po_x-source_po * cos(fovv * (double)(row - msg->height/2)/(double)(msg->height)) * cos ( fovh * (double)(step - msg->step/2)/(double)(msg->step)); 
			//source_po_y=source_po_y+source_po * cos(fovv * (row - msg->height/2)/msg->height) * sin ( fovh * (step - msg->step/2)/msg->step);
			//source_po_z=source_po_z+source_po * sin(fovv * (row - msg->height/2)/msg->height) ;
		//}
	//}
	
	//printf ("X: %f \n", source_po_x);
	//printf ("Y: %f \n", source_po_y);
	//printf ("Z: %f \n", source_po_z);
	//printf ("Decimals: %lu \n", ((msg->data.size())));
	
	//printf ("Decimals: %d \n", msg->data[640*480+50]);
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
	//for (uint32_t step=(msg->width-40) ; step <= msg->width ; ++step)
	//{
		//for (uint32_t step1=(msg->height-40) ; step1 <= msg->height ; ++step1)
		//{maxVal
			//msg->data[step*step1]=0;
		//}
	//}

	
	
	int threshold_value = 0;
	int threshold_type = 3;
	int const max_value = 255;
	int const max_type = 4;
	static int max_BINARY_value = 255;
	static int binary_tresh=58;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	int morph_elem = 2;
	int morph_size = 22;
	int morph_operator = 1;
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	static int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	int ilation_elem =2 ,max_elem=21 , dilation_elem=2 ,dilation_size=21;
	int const max_kernel_size = 21;
	
	int erosion_elem=2,erosion_size=21;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    double maxVal=7000.0;
    double minVal=450.0;
    
  
  
    
    createTrackbar( " Canny thresh:", "binary", &thresh, max_thresh, 0 );
    createTrackbar( " binary_tresh:", "binary", &binary_tresh, max_thresh, 0 );
   // minMaxLoc(cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
    cv::Mat blur_img ,binary;
    cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( blur_img, binary, element );

   cv::threshold( binary, binary, binary_tresh, 255,0);
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
   imshow ("origin binary",binary);
   
  // Since MORPH_X : 2,3,4,5 and 6
  int operation = morph_operator + 2;

  Mat pashm = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

  /// Apply the specified morphology operation
  morphologyEx( blur_img, binary, operation, pashm );
  
  imshow ("Morphology allgorithm" ,binary);
  
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   
   
    cv::threshold( binary, binary, binary_tresh, 255,0);
    imshow ("2 binary",binary);
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
 
 /*
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( binary, binary, element );
 
 //erosion allgorithm for ...!

   int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  
  /// Apply the erosion operation
  erode( binary, binary, element );
   
   
   
   */
   
    Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( binary, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


 /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
     
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }

    
    /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }





























	
    cv::imshow("Blur", blur_img);
    cv::imshow("binary", binary);
    imshow( "Contours", drawing );
    
    
    
    

     
     
     
//    imshow( "Dilation Demo", dilation_dst );

    cv::waitKey(1);

    //image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  
 };
 
 

   int main(int argc, char** argv)
   {
   printf( " ********************************************** \n" );
   printf( " ********************************************** \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *     This program debug and Develop by      * \n" );
   printf( " *          Mohammad Hossein Kazemi           * \n" );
   printf( " *        All Right reserved 2015-2016        * \n" );
   printf( " *      Email:Mhkazemi_engineer@yahoo.com     * \n" );
   printf( " *     AmirKabir University of Technology     * \n" );
   printf( " *    AUT-MAV AUTONUMUS AIRIAL VEHICLE TEAM   * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " ********************************************** \n" );
   printf( " ********************************************** \n" );
   
     ros::init(argc, argv, "image_converter");
     ImageConverter ic;
     ros::spin();
     return 0;
   }
