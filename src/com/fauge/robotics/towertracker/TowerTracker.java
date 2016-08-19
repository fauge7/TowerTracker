package com.fauge.robotics.towertracker;

import java.awt.Frame;
import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
	}
//	constants for the color rbg values
	public static final Scalar 
	RED = new Scalar(0, 0, 255),
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	BLACK = new Scalar(0,0,0),
	YELLOW = new Scalar(0, 255, 255),
//	these are the threshold values in order 
	LOWER_BOUNDS = new Scalar(0,0,95),
	UPPER_BOUNDS = new Scalar(100,255,255);
	
//	the size for resing the image
	public static final Size resize = new Size(320,240);
	
//	ignore these
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;
	
//	Constants for known variables
//	the height to the top of the target in first stronghold is 97 inches	
	public static final int TOP_TARGET_HEIGHT = 97;
//	the physical height of the camera lens
	public static final int TOP_CAMERA_HEIGHT = 17;
	
//	camera details, can usually be found on the datasheets of the camera
	public static final double VERTICAL_FOV  = 67;
	public static final double HORIZONTAL_FOV  = 54;
	public static final double CAMERA_ANGLE = 15;
	
	public static final double ROBOT_OFFSET_TO_FRONT = 21;
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
//		NetworkTable.setClientMode();
//		NetworkTable.setTeam(3019);
//		NetworkTable.setIPAddress("10.30.19.25");
//		NetworkTable.initialize();
//		NetworkTable table = NetworkTable.getTable("TowerTracker");
//		while(!table.isConnected()){}
//		main loop of the program
		int x = 0;
		while(x !0){
			try {
//				opens up the camera stream and tries to load it
				videoCapture = new VideoCapture();
				Mat mat = Imgcodecs.imread("http://192.168.1.22/image.jpg");
				Imgcodecs.imwrite("dlinktest.jpg", mat);
//				replaces the ##.## with your team number
//				videoCapture.open("http://10.##.##.11/mjpg/video.mjpg");
//				Example
//				videoCapture.open("http://192.168.1.22/image.jpg");
//				wait until it is opened
//				while(!videoCapture.isOpened()){}
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
	 * @param table the network table you want to output to
	 */
	public static void processImage(NetworkTable table){
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x,y,targetX,targetY,distance,azimuth;
//		frame counter
		int FrameCount = 0;
		long before = System.currentTimeMillis();
//		only run for the specified time
		
		while(FrameCount <5){
			contours.clear();
//			capture from the axis camera
			videoCapture.read(matOriginal);
//			captures from a static file for testing
//			matOriginal = Imgcodecs.imread("someFile.png");
			Imgproc.cvtColor(matOriginal,matHSV,Imgproc.COLOR_BGR2HSV);			
			Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
			Imgproc.CHAIN_APPROX_SIMPLE);
//			make sure the contours that are detected are at least 20x20 
//			pixels with an area of 400 and an aspect ration greater then 1
			for(MatOfPoint mop : contours){
				Rect rec = Imgproc.boundingRect(mop);
				Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), RED);
			}
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				Rect rec = Imgproc.boundingRect(matOfPoint);
					if(rec.height < 15 || rec.width < 15){
						iterator.remove();
					continue;
					}
					float aspect = (float)rec.width/(float)rec.height;
					if(aspect < 1.0)
						iterator.remove();
//					System.out.println( rec.height*rec.width);
				}
				
			table.putNumber("contours", contours.size());
			table.flush();
//			if there is only 1 target, then we have found the target we want
			if(contours.size() == 1){
				Rect rec = Imgproc.boundingRect(contours.get(0));
//				"fun" math brought to you by miss daisy (team 341)!
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
				table.putNumber("distance", distance-ROBOT_OFFSET_TO_FRONT);
//				System.out.println(azimuth);
				table.putNumber("azimuth", azimuth);
//				Imgproc.putText(matOriginal, ""+(int)distance, center, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
//				Imgproc.putText(matOriginal, ""+(int)azimuth, centerw, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
			}
//			output an image for debugging
			Imgcodecs.imwrite("output.png", matOriginal);
//			FrameCount++;
		}
//		shouldRun = false;
	}
	public static void processImage(){
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x,y,targetX,targetY,distance,azimuth;
//		frame counter
		int FrameCount = 0;
		long before = System.currentTimeMillis();
//		only run for the specified time
		
		while(FrameCount <5){
			contours.clear();
//			capture from the axis camera
			videoCapture.read(matOriginal);
//			captures from a static file for testing
//			matOriginal = Imgcodecs.imread("someFile.png");
			Imgproc.cvtColor(matOriginal,matHSV,Imgproc.COLOR_BGR2HSV);			
			Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
			Imgproc.CHAIN_APPROX_SIMPLE);
//			make sure the contours that are detected are at least 20x20 
//			pixels with an area of 400 and an aspect ration greater then 1
			for(MatOfPoint mop : contours){
				Rect rec = Imgproc.boundingRect(mop);
				Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), RED);
			}
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				Rect rec = Imgproc.boundingRect(matOfPoint);
					if(rec.height < 15 || rec.width < 15){
						iterator.remove();
					continue;
					}
					float aspect = (float)rec.width/(float)rec.height;
					if(aspect < 1.0)
						iterator.remove();
//					System.out.println( rec.height*rec.width);
				}
//			if there is only 1 target, then we have found the target we want
			if(contours.size() == 1){
				Rect rec = Imgproc.boundingRect(contours.get(0));
//				"fun" math brought to you by miss daisy (team 341)!
				y = rec.br().y + rec.height / 2.0;
				y= -((2 * (y / matOriginal.height())) - 1);
				distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) / 
						Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);
//				angle to target...would not rely on this
				targetX = rec.tl().x + rec.width / 2.0;
				targetX = (2 * (targetX / matOriginal.width())) - 1;
				azimuth = normalize360(targetX*HORIZONTAL_FOV /2.0 + 0);
//				drawing info on target
				Point center = new Point(rec.br().x-rec.width / 2.0 - 15,rec.br().y - rec.height / 2.0);
				Point centerw = new Point(rec.br().x-rec.width / 2.0 - 15,rec.br().y - rec.height / 2.0 - 20);
				Imgproc.putText(matOriginal, ""+(int)distance, center, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
				Imgproc.putText(matOriginal, ""+(int)azimuth, centerw, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
			}
//			output an image for debugging
			Imgcodecs.imwrite("output.png", matOriginal);
//			FrameCount++;
		}
//		shouldRun = false;
	}
	public static void trackBall(){
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Mat earlyFrame = new Mat();
		Mat currentFrame = new Mat();
		Mat diffFrame = new Mat();
		Mat grayFrame = new Mat();
		Mat blurFrame = new Mat();
		Mat threshFrame = new Mat();
//		frame counter
		int FrameCount = 0;
//		only run for the specified time
		while(FrameCount < 51){
			contours.clear();
//			capture from the axis camera
			
			
			if(FrameCount == 0){
				videoCapture.read(currentFrame);
				try{
					System.out.println(3);
					Thread.sleep(1000);
					System.out.println(2);
					Thread.sleep(1000);
					System.out.println(1);
					Thread.sleep(1000);
					System.out.println("go");
				} catch(Exception e){}
				
			}
			else{
				currentFrame.copyTo(earlyFrame);
				videoCapture.read(currentFrame);
				Core.absdiff(earlyFrame, currentFrame, diffFrame);
				Imgproc.cvtColor(diffFrame, grayFrame, Imgproc.COLOR_BGR2GRAY);
//				Imgproc.blur(grayFrame, blurFrame, new Size(2,2));
				Core.inRange(grayFrame, new Scalar(0), new Scalar(255), threshFrame);
				Imgproc.findContours(threshFrame, contours, new Mat(),Imgproc.RETR_EXTERNAL, 
						Imgproc.CHAIN_APPROX_SIMPLE);
				
				
				Imgcodecs.imwrite("contours" + FrameCount + ".png", diffFrame);
				Imgcodecs.imwrite("blur" + FrameCount+".png", blurFrame);
				for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
					MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
					Rect rec = Imgproc.boundingRect(matOfPoint);
					Imgproc.rectangle(diffFrame, rec.br(), rec.tl(), GREEN);
					
				}
				Imgcodecs.imwrite("difference" + FrameCount+ ".png", diffFrame);
				System.out.println(FrameCount + " done");
				
			}
			FrameCount++;
			shouldRun = false;
		
		}
	}
	/**
	 * @param angle a nonnormalized angle
	 */
	public static double normalize360(double angle){
		// Mod the angle by 360 to give a value between (0, 360]
		// Make it positive (by adding 360) if required
		return (angle < 0) ? angle % 360 + 360 : angle % 360;
	}
}
