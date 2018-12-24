#include "../flycapture.2.9.3.43_armhf/include/FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <wiringSerial.h>


using namespace FlyCapture2;
using namespace cv;
using namespace std;

ofstream dosyaYaz ("denemethr6.txt",ios::app); 

float errorX;
float errorY;
int posX = 0;
int posY = 0;

int midPositionThrottle = 0;
int midPositionRudder = 0;

int rudder = 0;
int aileron = 0;
int throttle = 0;
int elevator = 0;
  
int width = 100;
int height = 100;
int pErrorX = 0;
int pErrorY = 0;
int erroriX = 0;
int erroriY = 0;

float totalErrorX = 0;
float totalErrorY = 0;

float kd = 0;
float kp = 0;
float ki = 0;

float kdRudder = 0;
float kpRudder = 0;
float kiRudder = 0;

float kdThrottle = 0;
float kpThrottle = 0;
float kiThrottle = 0;
   
std::ofstream odroid;
    
float errorTot [2];

float pidCal (int a, int b);

int sendMsg (int throttle, int rudder, int elevator, int aileron);

int main()
{		
    Error error;
    Camera camera;
    CameraInfo camInfo;

	
    // Connect the camera
    error = camera.Connect( 0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;     
        return false;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;     
        return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " " 
              << camInfo.serialNumber << std::endl;
	
    error = camera.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;     
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;     
        return false;
    } 
	
	/*
	//HSV controll
	namedWindow ("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0; 
	int iHighS = 255;

	int iLowV = 0;	
	int iHighV = 255;

	
	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	//Create trackbars in "Control" window
	cvCreateTrackbar("Kp ", "Control", &kpc, 100); 
	cvCreateTrackbar("Ki ", "Control", &kic, 100); 
	cvCreateTrackbar("Kd ", "Control", &kdc, 100); 
	*/
	
	//PID control
	int kpc = 0;
	int kic = 0; 
	int kdc = 0;
	
	int iLastX = -1; 
	int iLastY = -1;
	

	
	//Capture a temporary image from the camera
	Mat imgTmp;
	//cap.read(imgTmp); 

	//Create a black image with the size as the camera output
	Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;

    // capture loop
    char key = 0;
    while(key != 'q')
    {	
		// Get the image
        Image rawImage;
        Error error = camera.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK )
        {
                std::cout << "capture error" << std::endl;
                continue;
        }
        
        // convert to rgb
        Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        cv::Mat imageOrj = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
			
		// image process
		//resize the image
		resize(imageOrj, imageOrj, Size(width,height), 0, 0, INTER_CUBIC);
		
		Mat imageHSV;
		cvtColor(imageOrj, imageHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		
		Mat imgThresholded
		;
		//inRange(imageHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		inRange(imageHSV, Scalar(62,32,62), Scalar(84, 255, 255), imgThresholded); //Threshold the image
      
		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (fill small holes in the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		
		//showing centre of ferame
		circle(imageOrj, Point(width/2,height/2), 1, Scalar (0,255,0), 2);
		
		//calculatıon of object center
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;
		
		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > width*100)
		{
			      
			//calculate the position of the ball
			posX = dM10 / dArea;
			posY = dM01 / dArea; 
			
			std::cout << "Point(x,y)=" << Point(posX, posY) << std::endl;       
			
			//error calculation PID coeff.
			kp = kpc * 0.1 ;
			ki = kic * 0.01;
			kd = kdc * 0.01;
			
			kpRudder = 4.3;
			kiRudder = 0.01;
			kdRudder = 0;
			
			kpThrottle =2;
			kiThrottle = 0.01;
			kdThrottle = 0;
			
			//std::cout << kpRudder<< "" << kdRudder<< std::endl; 
			
			pidCal (posX, posY);
			
			std::cout << totalErrorX << "" << totalErrorY << std::endl; 
			
			throttle = 1580; // mid value of throttle
			aileron = 1516; // mid value of aileron 
			//rudder = 1504;
			elevator = 1476;
			
			//calculation control signal to quadcopter if error > 30 pixel 
			if (errorX > width/20 || errorX < -1*width/20)
			{
				// X calculation [min-1100-mid-1504-max-1900]
				midPositionRudder = 1504;
				rudder = midPositionRudder + totalErrorX;
				std::cout << "X control signal " << rudder << std::endl;
				
			}
			
			if (errorY > height/20 || errorY < -1*height/20) 
			{
				// Y calculation [min-1100-mid-1504-max-1900]
				//int midPositionThrottle = 1504;
				//midPositionThrottle = 1625;
				//throttle = midPositionThrottle + totalErrorY;
				//throttle = throttle + totalErrorY;
				std::cout << "Y control signal " << throttle << std::endl;
				
			}
		
			// writing text 
			dosyaYaz<<midPositionThrottle<<" "<<throttle<<" "<<midPositionRudder<<" "<<rudder<<" "<<posX<<" "<<posY<<" "<<errorX<<" "<<errorY<<endl;
			// sending message to arduino
			sendMsg (throttle, rudder, elevator, aileron);
			
		
			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
			{
				//Draw a red circle from the previous point to the current point 
				line(imageOrj, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
				circle(imageOrj, Point(posX, posY), width/10 , Scalar (0,0,255), 1);
			}

			iLastX = posX;
			iLastY = posY;
		}
		else
		{
			throttle = 1504; // mid value of throttle
			aileron = 1516; // mid value of aileron 
			rudder = 1504;	// mid value of rudder
			elevator = 1476; 	// mid value of elevator
			
			// sending message to arduino
			sendMsg (throttle, rudder, elevator, aileron);
		} 
		
		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imageOrj); //show the original image

        key = cv::waitKey(40);        
    }

    error = camera.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }  

    camera.Disconnect();
	
	dosyaYaz<<"\n"<<endl;
    return 0;
}

float pidCal (int a, int b)
{

  //kp
  errorX = a - width/2;
  errorY = height/2 - b;
  //kd
  int errordX = errorX - pErrorX;
  int errordY = errorY - pErrorY;
  //ki
  erroriX += errorX;
  erroriY += errorY;
  
  //float errorTotX = errorX * kp + errordX * kd + erroriX * ki;
  float errorTotX = errorX * kpRudder + errordX * kdRudder + erroriX * kiRudder;
  float errorTotY = errorY * kpThrottle + errordY * kdThrottle + erroriY * kiThrottle;
  //float errorTotY = errorY * kp + errordY * kd + erroriY * ki;
  
  pErrorX = errorX;
  pErrorY = errorY;
  
  totalErrorX = errorTotX;
  totalErrorY = errorTotY;
  
  std::cout << "Error(x,y)=" << (totalErrorX) << "" << (totalErrorY) << std::endl;
  
  return 0;
}

int sendMsg (int throttle, int rudder, int elevator, int aileron)
{
	int maxThrottle = 1900;
	int minThrottle = 1110;
	int maxRudder = 1900;
	int minRudder = 1110;
	int maxElevator = 1740;
	int minElevator = 1260;
	int maxAileron = 1740;
	int minAileron = 1260;
	
	if (throttle > maxThrottle)
	{
		throttle = maxThrottle;
	}
	else if (throttle < minThrottle) 
	{
		throttle = minThrottle;
	}
	
	if (rudder > maxRudder)
	{
		rudder = maxRudder;
	}
	else if (rudder < minRudder) 
	{
		rudder = minRudder;
	}
	
	if (elevator > maxElevator)
	{
		elevator = maxElevator;
	}
	else if (elevator < minElevator) 
	{
		elevator = minElevator;
	}
	
	if (aileron > maxAileron)
	{
		aileron = maxAileron;
	}
	else if (aileron < minAileron) 
	{
		aileron = minAileron;
	}
	
	//total control signal
	std::cout << "control signals " << "\nT " << throttle << "\nR " << rudder << "\nE " << elevator << "\nA " << aileron << std::endl;
	
	//serial
	odroid.open( "/dev/ttyACM1");

	//write to it
    odroid << throttle << "t" << rudder << "r" << elevator << "e" << aileron << "a";
	
	odroid.close(); 

	return 0;	
}

