package org.firstinspires.ftc.teamcode.VisionBase;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
/*@Config*/
public class poleDistanceDetection extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 150;
    public static double strictHighS = 255;
    public double lArea = 0;
    public int cx, cy, fcx, fcy, zcx, zcy;
    public boolean inputtt, inputtt2 = false;
    public boolean poleDetected = false;
    Mat mat = new Mat();
    Mat output = new Mat();
    Mat input2 = new Mat();
    Mat thresh = new Mat();
    Mat masked = new Mat();
    Mat scaledMask = new Mat();
    Mat scaledThresh = new Mat();
    Mat selection = new Mat();
    Mat Selection = new Mat();
    Mat fSelect = new Mat();
    Mat selection2 = new Mat();
    Mat Selection2 = new Mat();
    Mat fSelect2 = new Mat();
    public int contourNo;
    public poleDistanceDetection() {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(output);
        input.copyTo(input2);

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(15, 30, 20); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(28, 255, 255); // lenient higher bound HSV for yellow
        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);


        Moments M = new Moments();
        M =Imgproc.moments(scaledThresh);
        cx = (int)(M.m10/M.m00);
        cy = (int)(M.m01/M.m00);
        Imgproc.rectangle(input, new Point((cx-125), 0), new Point((cx+125), 448), new Scalar(255, 255, 255), -1);
        Imgproc.cvtColor(input, Selection, Imgproc.COLOR_RGB2HSV);
        Core.inRange(Selection, new Scalar(0, 0, 255), new Scalar(0, 0, 255), selection);
        Core.bitwise_and(scaledThresh, scaledThresh, fSelect, selection);
//        Core.bitwise_and(scaledThresh, scaledMask, contour, selection);
//        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want

        Moments f = new Moments();
        f =Imgproc.moments(fSelect);
        fcx = (int)(f.m10/f.m00);
        fcy = (int)(f.m01/f.m00);
        if (frameList.size() > 5) {
            frameList.remove(0);
        }
        Imgproc.rectangle(input2, new Point((fcx-50), 0), new Point((fcx+50), 448), new Scalar(255, 255, 255), -1);
        Imgproc.cvtColor(input2, Selection2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(Selection2, new Scalar(0, 0, 255), new Scalar(0, 0, 255), selection2);
        Core.bitwise_and(scaledThresh, scaledThresh, fSelect2, selection2);
        Moments z = new Moments();
        z =Imgproc.moments(fSelect2);
        zcx = (int)(z.m10/z.m00);
        zcy = (int)(z.m01/z.m00);
        if (frameList.size() > 5) {
            frameList.remove(0);
        }
        if(Core.countNonZero(fSelect2)>6000){
            poleDetected = true;
        Imgproc.line(output, new Point(zcx, 600), new Point(zcx, 0), new Scalar(0, 255, 255));}
        else poleDetected=false;
//        Imgproc.line(scaledThresh, new Point(cx, 448), new Point(cx, 0), new Scalar(0, 255, 255));
        //release all the data
//        input.release();
        if(inputtt){
            fSelect2.copyTo(output);}
        if(inputtt2){
            selection.copyTo(output);}
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        thresh.release();
        selection.release();
        Selection.release();
        fSelect.release();
        selection2.release();
        Selection2.release();
        fSelect2.release();
//        input.release();
        input2.release();

        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return output;
    }
    public int getDistance(){
        if(poleDetected){
        return (400-zcx);}
        else return(1000);
    }



}