
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
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("10.30.19.25");
	}
//	constants for the color rbg values
	public static final Scalar 
	RED = new Scalar(0, 0, 255),
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	BLACK = new Scalar(0,0,0),
	YELLOW = new Scalar(0, 255, 255),
//	these are the threshold values in order (HSV)
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
	public static final double CAMERA_ANGLE = 17.5;
	
//  shooter settings
	public static final double SHOOTER_SPEED = 30.806;//launch speed
	public static final double OFFSET_TO_FRONT = 0;
	
	public static boolean shouldRun = true;

	/**
	 * 
	 * @param args command line arguments0
	 * just the main loop for the program and the entry points
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		matOriginal = new Mat();
		matHSV = new Mat();
		matThresh = new Mat();
		clusters = new Mat();
		matHeirarchy = new Mat();
		NetworkTable table = NetworkTable.getTable("TowerTracker");
		System.out.println("Connected to table " + table.isConnected());
//		main loop of the program
		while(shouldRun){
			try {
//				opens up the camera stream and tries to load it
				videoCapture = new VideoCapture();
//				replaces the ##.## with your team number
				videoCapture.open("http://10.30.19.11/mjpg/video.mjpg");
//				Example
//				cap.open("http://10.30.19.11/mjpg/video.mjpg");
//				wait until it is opened
				while(!videoCapture.isOpened()){}
//				time to actually process the acquired images
				processImage(table);
				
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
	public static void processImage(NetworkTable table){
		System.out.println("processed");
		
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x,y,targetX,targetY,distance ,azimuth;
		
//		frame counter
		int FrameCount = 0;
		long before = System.currentTimeMillis();
//		only run for the specified time
		while(true){
//			System.out.println("RAN");
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
				}
				for(MatOfPoint mop : contours){
					Rect rec = Imgproc.boundingRect(mop);
					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
			}
				table.putNumber("VISnumContours", contours.size());
//			if there is only 1 target, then we have found the target we want
			if(contours.size() == 2){
					Rect rec1 = Imgproc.boundingRect(contours.get(0));
					Rect rec2 = Imgproc.boundingRect(contours.get(1));
					if(rec1.area() > rec2.area()){
					contours.remove(1);	
					}
					else{
						contours.remove(0);
					}
			}
				if(contours.size() == 1){
				Rect rec = Imgproc.boundingRect(contours.get(0));
//				"fun" math brought to you by miss daisy (team 341)!
				y = rec.br().y + rec.height / 2;
				y= -((2 * (y / matOriginal.height())) - 1);
				distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) / 
						Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);
//				angle to target, Pretty good
				targetX = rec.tl().x + rec.width / 2;
				targetX = (2 * (targetX / matOriginal.width())) - 1;
				azimuth = normalize360(targetX*HORIZONTAL_FOV /2.0 + 0);
				
				System.out.println(table.getString("TEST"));
				
//				distance to angle bullshit
				
				double beta = distance;//distance in inches from target
				double alpha = SHOOTER_SPEED;//shoot speed

				double[] radians = quad(
						/*term a*/	(-0.003165*Math.pow(beta, 2))/(Math.pow(alpha, 2)),		
						/*term b*/	(beta/39.37),		
						/*term c*/	(-(2.0+((0.003165*Math.pow(beta, 2))/(Math.pow(alpha, 2)))))
									);
				double radian1 = radians[0];

//				example for finding the angle a shooter needs to be at for the the given distance
//				to find the equation find 3 point that the shooter make the boulder at known distances
//				and plug them in to a quadratic fit on wolfram alpha using (angle,distance)
				double targetAngle = 0.00866306 * (distance*distance) - 2.28831 * (distance) + 176.091;
				System.out.println("target Angle1 = " + targetAngle);

//				outputting to network table
				table.putNumber("targetAngle", targetAngle);
				table.putNumber("VISdistance", distance + OFFSET_TO_FRONT);
				table.putNumber("VISazimuth", azimuth);
				table.putNumber("VISnumCycles", FrameCount);

//				drawing info on target
				Point center = new Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2);
				Point centerw = new Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2 - 20);
				Imgproc.putText(matOriginal, ""+(int)distance, center, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
				Imgproc.putText(matOriginal, ""+(int)azimuth, centerw, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
			} 
//			output an image for debugging
//			Imgcodecs.imwrite("output.png", matOriginal);
			FrameCount++;
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
	public static double[] quad(double a, double b, double c){
		 
        //Finding out the roots
        double temp1 = Math.sqrt(b * b - 4 * a * c);
 
        double root1 = (-b +  temp1) / (2*a) ;
        double root2 = (-b -  temp1) / (2*a) ;
        double rad1 = Math.atan(root1);
        double rad2 = Math.atan(root2);

        double[] answers = {rad1,rad2};
        return answers;
 
	}

}