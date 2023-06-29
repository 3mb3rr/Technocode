package org.firstinspires.ftc.teamcode.Mech.subsystems;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class ChassisSubsystem extends SubsystemBase {
    public boolean auto = true;
    public Pose2d robotPos = new Pose2d(0, 0, 0);
    public Pose2d expectedPos=robotPos;
    public boolean holdingTemp = true; // its a random variable used for the holding(insignificant)
    public Trajectory t1;
    double x, y, turn;
    double xf, yf, turnf;
    double lastTime = -1;
    boolean trajectoryCompleted = true;
    SampleMecanumDrive drive;
    public enum chassis{
        driving, holding, tDriving
    }
    public ElapsedTime timer = new ElapsedTime();
    public chassis chassisState = chassis.holding;
    public ChassisSubsystem(final HardwareMap hMap) {
        register();
        drive = new SampleMecanumDrive(hMap);
    }


    public void moveTo(Pose2d targetPos1){
        expectedPos=targetPos1;
        t1 = drive.trajectoryBuilder(robotPos)
                .lineToLinearHeading(targetPos1)
                .addDisplacementMarker(() -> {
                    trajectoryCompleted = true;
                })
                .build();
        trajectoryCompleted = false;
        drive.followTrajectoryAsync(t1);
    }
    public void moveTo(Pose2d targetPos1, Pose2d targetPos2){
        expectedPos=targetPos2;
        Trajectory t1 = drive.trajectoryBuilder(robotPos)
                .lineToLinearHeading(targetPos1)
                .lineToLinearHeading(targetPos2)
                .addDisplacementMarker(() -> {
                    trajectoryCompleted = true;
                })
                .build();
        trajectoryCompleted = false;
        drive.followTrajectoryAsync(t1);
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


    @Override
    public void periodic(){
        drive.update();
        robotPos = drive.getPoseEstimate();
        switch(chassisState) {
            case holding:
                if((timer.milliseconds()>(lastTime+500)) && (trajectoryCompleted)){
                    lastTime= timer.milliseconds();
                if(holdingTemp){moveTo(new Pose2d((egetX()+0.000001), egetY(), egetHeading()));}
                else{moveTo(expectedPos);}}

        }

    }
}