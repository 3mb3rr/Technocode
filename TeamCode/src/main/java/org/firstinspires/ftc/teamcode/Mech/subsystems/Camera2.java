package org.firstinspires.ftc.teamcode.Mech.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionBase.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.VisionBase.poleDistanceDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera2 extends SubsystemBase {
    OpenCvCamera camera, ttCamera;
    public static AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public static poleDistanceDetection poleDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    public int parkingZone = 0;
    public boolean inSight;

    public Camera2(final HardwareMap hMap) {
        register();
        int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
        ttCamera = OpenCvCameraFactory.getInstance().createWebcam(hMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        poleDetectionPipeline = new poleDistanceDetection();
        ttCamera.setPipeline(poleDetectionPipeline);
        ttCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                ttCamera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(ttCamera, 30);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

    }
    public int getPoleError(){
        return poleDetectionPipeline.getDistance();
    }



    @Override
    public void periodic(){

    }
}