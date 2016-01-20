package com.fauge.robotics.towertracker;

import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * 
 * @author Elijah Kaufman
 * @version 1.0
 * @description Uses opencv and network table 3.0 to detect the vision targets
 *
 */
public class TowerTracker {

	/**
	 * static method to load opencv and networkTables
	 */
	static{ 
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("roborio-3019.local");
	}
//	constants for the color rbg values
	public static final Scalar 
	RED = new Scalar(0, 0, 255),
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	BLACK = new Scalar(0,0,0),
	YELLOW = new Scalar(0, 255, 255),
//	these are the threshold values in order 
	LOWER_BOUNDS = new Scalar(0,0,165),
	UPPER_BOUNDS = new Scalar(180,255,255);
	
//	ignore these
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;

//	Constants for known variables
//	the height to the top of the target in first stronghold is 97 inches	
	public static final int TOP_TARGET_HEIGHT = 97;
//	the physical height of the camera lens
	public static final int TOP_CAMERA_HEIGHT = 32;
	
//	camera details, can usually be found on the datasheets of the camera
	public static final double VERTICAL_FOV  = 51;
	public static final double HORIZONTAL_FOV  = 67;
	public static final double CAMERA_ANGLE = 10;
	
	public static boolean shouldRun = true;

	/**
	 * 
	 * @param args command line arguments
	 * just the main loop for the program and the entry points
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		matOriginal = new Mat();
		matHSV = new Mat();
		matThresh = new Mat();
		clusters = new Mat();
		matHeirarchy = new Mat();
		NetworkTable table = NetworkTable.getTable("SmartDashboard");
//		main loop of the program
		while(shouldRun){
			try {
//				opens up the camera stream and tries to load it
				videoCapture = new VideoCapture();
//				replaces the ##.## with your team number
				videoCapture.open("http://10.##.##.11/mjpg/video.mjpg");
//				Example
//				cap.open("http://10.30.19.11/mjpg/video.mjpg");
//				wait until it is opened
				while(!videoCapture.isOpened()){}
//				time to actually process the acquired images
				processImage();
			} catch (Exception e) {
				e.printStackTrace();
				break;
			}
		}
//		make sure the java process quits when the loop finishes
		videoCapture.release();
		System.exit(0);
	}
	/**
	 * 
	 * reads an image from a live image capture and outputs information to the SmartDashboard or a file
	 */
	public static void processImage(){
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x,y,targetX,targetY,distance,azimuth;
//		frame counter
		int FrameCount = 0;
		long before = System.currentTimeMillis();
//		only run for the specified time
		while(FrameCount < 100){
			contours.clear();
			videoCapture.read(matOriginal);
			Imgproc.cvtColor(matOriginal,matHSV,Imgproc.COLOR_RGB2HSV);			
			Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
					Imgproc.CHAIN_APPROX_SIMPLE);
//			make sure the contours that are detected are at least 20x20 
//			pixels with an area of 400 and an aspect ration greater then 1
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				Rect rec = Imgproc.boundingRect(matOfPoint);
					if(rec.height*rec.width < 400 || rec.height < 20 || rec.width < 20){
						iterator.remove();
					continue;
					}
					float aspect = (float)rec.width/(float)rec.height;
					if(aspect < 1.0)
						iterator.remove();
				}
				for(MatOfPoint mop : contours){
					Rect rec = Imgproc.boundingRect(mop);
					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
			}
//			if there is only 1 target, then we have found the target we want
			if(contours.size() == 1){
				Rect rec = Imgproc.boundingRect(contours.get(0));
//				"fun" math
				y = rec.br().y + rec.height / 2;
				y= -((2 * (y / matOriginal.height())) - 1);
				distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) / 
						Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);
//				angle to target...would not rely on this
				targetX = rec.tl().x + rec.width / 2;
				targetX = (2 * (targetX / matOriginal.width())) - 1;
				azimuth = normalize360(targetX*HORIZONTAL_FOV /2.0 + 0);
//				drawing info on target
				Point center = new Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2);
				Point centerw = new Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2 - 20);
				Imgproc.putText(matOriginal, ""+(int)distance, center, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
				Imgproc.putText(matOriginal, ""+(int)azimuth, centerw, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
			}
//			output an image for debugging
			Imgcodecs.imwrite("output.png", matOriginal);
			FrameCount++;
		}
		shouldRun = false;
	}
	/**
	 * @param angle a nonnormalized angle
	 */
	public static double normalize360(double angle){
		while(angle >= 360.0)
        {
            angle -= 360.0;
        }
        while(angle < 0.0)
        {
            angle += 360.0;
        }
        return angle;
	}
}
