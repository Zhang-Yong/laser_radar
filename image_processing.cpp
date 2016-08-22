/*
 * image_processing.cpp
 *
 *  Created on: 2016年7月8日
 *      Author: Zhang Yong
 */

//include file for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include<vector>
#include "opencv2/opencv.hpp"
#include <unistd.h>

using std::vector;
using namespace std;
using namespace cv;

//define vector_pix_value as global to use by filter_outlier function
vector<int> vector_pix_value;   // use to get laser line
vector<int>vector_pix_row; 	   // use to calculate distance
vector<double>vector_distance;	// use to keep distance
int image_height;			// use to calculate distance

// optimize:  cut image on/off into half height after split red function, all image processing later on only work on half img,
// after get laser line func, we got laser loction, do not need to write img in get laser line and filter function.

//----------------------------------------Function Define Block  ------------------------------------------------------//

// input 3 channel normalized image, output red channel image
		void split_red_channel(Mat &image, Mat &red_channel_image)
		{
				vector<Mat> bgr_channels(3);
				split(image,bgr_channels);
				red_channel_image = bgr_channels[2];
		}

		void cut_image_half_height( Mat &red_channel_image)
		{
			//assume laser mount on Top of camera,  any data below middle line can be ignored.  Rect : x, y , image.width, image.height
			image_height = red_channel_image.rows;
			red_channel_image = Mat(red_channel_image, cv::Rect(0,0,red_channel_image.cols, image_height/2));
		}


		// optimize normalize function to use LUT to save time
		//LUT can use same image for input and output , but the size has to be 256
		void normalize_optimize_LUT(Mat &image)
		    {
					//get min max value of img
					/// Localizing the best match with minMaxLoc
					double minVal; double maxVal; //Point minLoc; Point maxLoc; //Point matchLoc;
					minMaxLoc( image, &minVal, &maxVal);
					double offset_factor = 255/( maxVal -minVal);

					// Create loop up table for image value, value store in table[i], i start from minVal to MaxVal
					uchar table[256];
					for (int i = 0; i < 256; ++i)
					{
					   table[i] = offset_factor* (i - minVal);
					}

					// Create lookupTable Matrix, 1 row, 256 cols, 8 bit unsigned char, LUT table has to be 256 size
					// otherwise: OpenCV Error: Assertion failed ((lutcn == cn || lutcn == 1) ...
					Mat lookUpTable(1, 256, CV_8U);
					uchar* p = lookUpTable.data;
					for( int i = 0; i < 256; ++i)
					{
						p[i] = table[i];
					}

					//Finally call lookup table
					 LUT(image, lookUpTable, image);

		 } //end of normalize function




		//This time need return reference call
		void diff_red_channel(Mat &image_on, Mat &image_off,Mat &diff_image)
		{
			// subtract is better than absdiff
			  subtract(image_on,image_off,diff_image);
		}


		void GaussianBlur (Mat &diff_image, Mat &diff_image2)
		{
			//Guassian Blur size is very import to filter those noise pix,  big size has better noise filter !
			// size too big will make laser line blur

			 GaussianBlur( diff_image, diff_image2, Size( 11, 11 ), 0, 0 );
		}

		 // Method: Init a black pic with all dots  0 ( already in main function), init a row array with all 0, scan image row by row
		// replace pix in row array with brighter pix value, keep (x,y) axis of these pixes.
		// CV function minMaxLoc fits this purpose,  it supports column max value and location
		void get_laser_line_optimize(Mat &diff_image2, Mat &laser_image)
		{
			  double  maxVal = 0;
			  Point maxLoc;
			  int nCols = diff_image2.cols;
			  for( int x = 0; x < nCols; x++ )
			  {
				  //minMaxLoc support cols
				  minMaxLoc( diff_image2.col(x),  NULL, &maxVal, NULL, &maxLoc);
				  // write laser image here
				  laser_image.at<uchar>(maxLoc.y, x) = maxVal;
				  // write pix value vector for later use
				  vector_pix_value.push_back(maxVal);
				  // write pix location to calculate distance
				  vector_pix_row.push_back(maxLoc.y);
     		   }//end for

		}// end function


		// To calculate distance, do not need to generate laser image
		void filter_outliers_LUT_Optimize(Mat &laser_image)
				{
					double sum = 0.0;
					for (vector<int>::size_type ix = 0; ix != vector_pix_value.size(); ++ix)
						 sum += vector_pix_value[ix];

					double mean =  sum / vector_pix_value.size();
					double accum = 0.0;
					double sigma = 0.0;
					double stdev = 0.0;
					// get Gaussian distribution of stdev & sigma
					for (vector<int>::size_type ix = 0; ix != vector_pix_value.size(); ++ix)
					{
						sigma =  vector_pix_value[ix] - mean;
						accum += sigma*sigma;
					}
					stdev = sqrt(accum / (vector_pix_value.size()-1));
					// 0.3 be able to filter image10, a key factor to filter background noise, smaller no. will filter more noise and laser data
					double outlier_level = mean - 0.3*stdev;

					// ignore outlier pix here, set both value and location to -1
					for (vector<int>::size_type ix = 0; ix != vector_pix_value.size(); ++ix)
					{
						if (vector_pix_value[ix] < outlier_level)
						{
							vector_pix_value[ix] = -1;
							vector_pix_row[ix] = -1;
						}
					}

					// Create loop up table for to create final laser image, if only need to get distance, comment code below
					// i is value, table[i] is mapping
					uchar table[256];
					for (int i = 0; i < 256; ++i)
					{
						if (i < outlier_level)
						{
							table[i] = 0;
						}
						else if ( i > 0 )
						{
							table[i] = 255;
						}
					}

					Mat lookUpTable(1, 256, CV_8U);
					uchar* p = lookUpTable.data;
					for( int i = 0; i < 256; ++i)
					{
						p[i] = table[i];
					}
					//Finally call lookup table
					 LUT(laser_image, lookUpTable, laser_image);
				}

			void get_distance()
			{
					// follow previous project parameter
					double rpc=0.00190576697085, ro= -0.0289569496793, h=20;
					for (vector<int>::size_type ix = 0; ix != vector_pix_row.size(); ++ix)
					{
							if ( vector_pix_row[ix] < 0)
							{
								vector_distance.push_back(-1);
							}
							else
							{
								double pfc = abs(vector_pix_row[ix] - image_height/2);
								double theta = rpc * pfc + ro;
								double distance = h/tan(theta);
								vector_distance.push_back(distance);
							}

					}
				for(int i = 0;  i != vector_distance.size(); ++i)
				{
					cout << vector_distance[i] <<",";
				}

				// as thread keep running, clear all vector at the end of function
				vector_distance.clear();
				vector_pix_row.clear();
				vector_pix_value.clear();

			}





