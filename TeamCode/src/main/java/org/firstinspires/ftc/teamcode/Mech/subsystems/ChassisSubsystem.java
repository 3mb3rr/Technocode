package org.firstinspires.ftc.teamcode.Mech.subsystems;


import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class ChassisSubsystem extends SubsystemBase {
    public boolean auto = true;
    public Pose2d robotPos = new Pose2d(0, 0, 0);
    public Pose2d expectedPos=robotPos;
    public boolean holdingTemp = true; // its a random variable used for the holding(insignificant)
    public Trajectory t1;
    double x, y, turn;
    double xf, yf, turnf;
    double lastTime = -1;
    public boolean trajectoryCompleted = false;
    public boolean holdCompleted = true;
    SampleMecanumDrive drive;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    public double xInput = 0;
    public double yInput = 0;
    public double turnInput = 0;


    public enum chassis{
        driving, holding, tDriving
    }

    public ElapsedTime timer = new ElapsedTime();
    public chassis chassisState = chassis.holding;
    public boolean BLorRR = false;
    public ChassisSubsystem(final HardwareMap hMap) {
        register();
        drive = new SampleMecanumDrive(hMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }


    public void moveTo(Pose2d targetPos1){
        trajectoryCompleted = false;
        expectedPos=targetPos1;
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(robotPos)
                .lineToLinearHeading(targetPos1)
                .addDisplacementMarker(() -> {
                    trajectoryCompleted = true;
                })
                .build();
        drive.followTrajectorySequenceAsync(t1);
    }
    public void holdTo(Pose2d targetPos1){
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(robotPos)
                .lineToLinearHeading(targetPos1)
                .addDisplacementMarker(() -> {
                    holdCompleted = true;
                })
                .build();
        holdCompleted = false;
        drive.followTrajectorySequenceAsync(t1);
    }
    public void moveTo(Pose2d targetPos1, Pose2d targetPos2){
        expectedPos=targetPos2;
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(robotPos)
                .resetVelConstraint()
                .lineToLinearHeading(targetPos1)
                .lineToLinearHeading(targetPos2)
                .addDisplacementMarker(() -> {
                    trajectoryCompleted = true;
                })
                .build();
        trajectoryCompleted = false;

        drive.followTrajectorySequenceAsync(t1);
    }
    public void moveTo(Pose2d targetPos1, double degrees){
        expectedPos= new Pose2d(targetPos1.getX(), targetPos1.getY(), (targetPos1.getHeading())+Math.toRadians(degrees));
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(robotPos)
                .resetVelConstraint()
                .lineToLinearHeading(targetPos1)
                .turn(Math.toRadians(degrees))
//                .addSpatialMarker(new Vector2d(targetPos1.getX(), targetPos1.getY()), () -> {
//                    trajectoryCompleted = true;
//                })
                .addDisplacementMarker(() ->
                {trajectoryCompleted = true;})
                .build();
        trajectoryCompleted = false;
        drive.followTrajectorySequence(t1);
    }

    public boolean atCorrectPosition(){
        if((getX()<(egetX()+0.5)) && (getX()>(egetX()-0.5)) && (getY()>(egetY()-0.5)) && (getY()<(egetY()+0.5)) && (getHeading()>(egetHeading()-0.7))&& (getHeading()<(egetHeading()+0.7))){
                return true;
        }
        return false;
    }
    public double getX(){
        return robotPos.getX();
    }
    public double getY(){
        return robotPos.getY();
    }
    public double getHeading(){
        return Math.toDegrees(robotPos.getHeading());
    }
    public double egetX(){
        return expectedPos.getX();
    }
    public double egetY(){
        return expectedPos.getY();
    }
    public double egetHeading(){
        return Math.toDegrees(expectedPos.getHeading());
    }
    public double getXVelocity(){
        return drive.getPoseVelocity().getX();
    }
    public double getYVelocity(){
        return drive.getPoseVelocity().getY();
    }
    public double getHeadingVelocity(){
        return drive.getPoseVelocity().getHeading();
    }

    public void brake() {

        drive.setMotorPowers(0, 0, 0, 0);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    @Override
    public void periodic(){
        drive.update();
        robotPos = drive.getPoseEstimate();
        switch(chassisState) {
            case holding:
                if((timer.milliseconds()>(lastTime+500)) && (holdCompleted) && (!atCorrectPosition())){
                    lastTime= timer.milliseconds();
                if(holdingTemp){holdTo(new Pose2d((egetX()+0.000001), egetY(), Math.toRadians(egetHeading())));}
                else{holdTo(expectedPos);}}

        }
        if(!auto){
            chassisState = chassis.driving;
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double r = Math.hypot(yInput, xInput);
            double robotAngle = Math.atan2(yInput, xInput) - Math.PI / 4  - ((angles.firstAngle/180)*Math.PI);
            double rightX = (turnInput)/1.35;
            final double v1 = (r * Math.cos(robotAngle) + rightX);
            final double v2 = (r * Math.sin(robotAngle) + rightX);
            final double v3 = (r * Math.cos(robotAngle) - rightX);
            final double v4 = (r * Math.sin(robotAngle) - rightX);

            if (yInput != 0 || xInput != 0 || turnInput != 0) {

                drive.setMotorPowers(v1, v2, v3, v4);
            }
            else{
                brake();

            }
        }

    }
}