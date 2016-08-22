/*
 * main.cpp
 *
 *  Created on: 2016年7月7日
 *      Author: Zhang Yong
 */

#include <stdio.h>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include<sstream>
#include <iostream>
#include<vector>

using std::vector;
using namespace std;
using namespace cv;

Mat image_on_clone, image_off_clone;   //define global Mat so that it can be used in diff thread
Mat image_on, image_off,laser_image;


//-------------------------------------------------- Program flow---------------------------------------------------------------------------//
//Set laser on ->grab image on (thread 0)-> trig  copy image on thread (thread 1) --> set laser off -->
// grab image off -> set laser on  -> trig image processing thread ( thread 2 )


#define NTHREADS 3
void *thread_grab_img(void *);
void *thread_set_laser_off_copy_img(void *);
void *thread_process_img(void *);
bool grab_image_on_finish = false; // initialize to false
bool grab_image_off_finish = false;
bool set_laser_off_grab_image_off = false;
// define conditional thread here
pthread_mutex_t condition_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  condition_grab_on_finish  = PTHREAD_COND_INITIALIZER;
pthread_cond_t  condition_grab_off_finish  = PTHREAD_COND_INITIALIZER;


//----------------------setup of read img from camera ------------------------------------------------------//
//put it here so that this can be used in thread
VideoCapture cap(0); // open the default camera

//----------------------setup of read img from camera ------------------------------------------------------//
//Time Zero used as reference to check thread execution time
double t0 = (double)getTickCount();
double t_grab_img_start ;
int counter =0;


int main(int argc, char** argv )
{

		//-----------------------camera setup ----------------------//
		if(!cap.isOpened())  // check if we succeeded
		{
			cout << "error, no camera exist " << endl;
			return -1;
		}
		//set frame rate to 30, this save image grab time
		cap.set(CV_CAP_PROP_FPS,  30);
//		//	    // Set exposure and focus manually.
//	    // Seems to take effect after several frames.
	    Mat temp_frame;
	    cap >> temp_frame;
	    temp_frame.release();
	    system("v4l2-ctl -d 0 -c exposure_auto=1");
	    system("v4l2-ctl -d 0 -c exposure_absolute=180");
	    system("v4l2-ctl -d 0 -c focus_auto=0");
	    system("v4l2-ctl -d 0 -c focus_absolute=0");
	   // --------------finish camera setup ---------------------------//



	   //  GPIO setup, put slow function out of thread loop-//
		void setup_io();
		setup_io();
		void set_laser_on();   //function declare
		void set_laser_off();   //function declare

	  pthread_t thread_id[NTHREADS];
	  pthread_create( &thread_id[0], NULL, thread_grab_img, NULL );
	  pthread_create( &thread_id[1], NULL, thread_set_laser_off_copy_img, NULL );
	  pthread_create( &thread_id[2], NULL, thread_process_img, NULL );

	  pthread_join( thread_id[0], NULL);  // wait grab on img to finish
	  pthread_join( thread_id[1], NULL);
	  pthread_join( thread_id[2], NULL);

      return 0;
} // end main

// grab image_on and off, after off, set laser on
void *thread_grab_img(void *dummyPtr)
{
	int image_counter = 0;
	 void set_laser_on();   //function declare, function define in laser_control.cpp
	 void set_laser_off();   //function declare


	 // clean up frame buffer here
	 for ( int i =0; i != 10; ++i)
	 {
		set_laser_on();
		cap >> image_on;
		set_laser_off();
		cap >> image_off;
	 }

	for(;;)
	{

		 //----------------------------function of read img from camera /file--------------------------------------------------//
		//read img and convert to gray scale, 0 means grayscale img , 1 is color

//		    image_on = imread( "on.jpg" , CV_LOAD_IMAGE_COLOR );
//			image_off = imread( "off.jpg" , CV_LOAD_IMAGE_COLOR );


		    set_laser_on();
		    usleep(1000);	 		//wait 1 ms

		 	//read img from camera
		    cap >> image_on; // get a new frame from camera

		   grab_image_on_finish = true; // conditional thread variable to trig image copy thread

		   // send signal to set laser off and copy img thread
		   pthread_mutex_lock( &condition_mutex );
		   if (grab_image_on_finish == true)
		   {
			  pthread_cond_signal( &condition_grab_on_finish );
		   }
		   pthread_mutex_unlock( &condition_mutex );
		   grab_image_on_finish = false;


		   set_laser_off();
		   usleep(1000);		// wait 1 ms

		   cap >> image_off; // get a new frame from camera

		   grab_image_off_finish = true;

		   // send signal to  img processing thread after grab img off
		   pthread_mutex_lock( &condition_mutex );
		  if (grab_image_off_finish == true)
		  {
		   pthread_cond_signal( &condition_grab_off_finish );
		  }
		  pthread_mutex_unlock( &condition_mutex );

		  grab_image_on_finish = false;
		  grab_image_off_finish = false;

		 usleep(20000);	//wait 15 ms before set laser on, not necessary if we have a quick turn off laser or circuit

	} // for loop

}


// this function supposed to execute after grab image_on finish
void *thread_set_laser_off_copy_img(void *dummyPtr)
{

	for(;;)
	  {
	      pthread_mutex_lock( &condition_mutex );
	      pthread_cond_wait( &condition_grab_on_finish, &condition_mutex );
          // copy image on here to avoid overwrite by next frame during image processing function
	      image_on_clone = image_on.clone();
	      pthread_mutex_unlock( &condition_mutex );
	  } // for loop
}

void *thread_process_img(void *dummyPtr)
{
			// split red channel & convert 3 channel to single channel img
			Mat red_channel_image_on = Mat::zeros(image_on_clone.size(), CV_8UC1);
			Mat red_channel_image_off = Mat::zeros(image_off.size(), CV_8UC1);

			//function declare here
			void split_red_channel(Mat &image, Mat &red_channel_image);
			void cut_image_half_height( Mat &red_channel_image);
		    void normalize_optimize_LUT(Mat &image);
		    void diff_red_channel(Mat &image_on, Mat &image_off, Mat &diff_image);
		    void GaussianBlur (Mat &diff_image, Mat &diff_image2);
		    void get_laser_line(Mat &diff_image2, Mat &laser_image);
		    void get_laser_line_optimize(Mat &diff_image2, Mat &laser_image);
		    void filter_outliers(Mat &laser_image);
		    void filter_outliers_LUT_Optimize(Mat &laser_image);
		    void get_distance();
		    int image_counter = 0;

	for(;;)
		  {
		// wait for grab off thread finish
		  pthread_mutex_lock( &condition_mutex );
		  pthread_cond_wait( &condition_grab_off_finish, &condition_mutex );

			// ----------------------Image processing Function call here ----------------------------------------------------//
			//copy img off here to avoid overwrite by next frame
			image_off_clone = image_off.clone();

			 split_red_channel(image_on_clone, red_channel_image_on);
			 split_red_channel(image_off_clone,red_channel_image_off);

			 cut_image_half_height(red_channel_image_on);
			 cut_image_half_height(red_channel_image_off);

			  normalize_optimize_LUT(red_channel_image_on);
			  normalize_optimize_LUT(red_channel_image_off);

			 Mat diff_image;
			 diff_red_channel(red_channel_image_on, red_channel_image_off, diff_image);

			 Mat diff_image2 = Mat::zeros(diff_image.size(), CV_8UC1);
			 GaussianBlur (diff_image, diff_image2);

		   //call get laser line
			laser_image = Mat::zeros(diff_image2.size(), CV_8UC1);
			get_laser_line_optimize(diff_image2, laser_image);

			filter_outliers_LUT_Optimize(laser_image);

			 // call get distance, ignore first 20 images
			 if (image_counter > 20 )
			 {
				 get_distance();
			 }
			 else
				 image_counter ++;

			 pthread_mutex_unlock( &condition_mutex );

		  } // for loop

	}

