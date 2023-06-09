package org.firstinspires.ftc.teamcode.Mech.subsystems;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class ChassisSubsystem extends SubsystemBase {
    public boolean auto = true;
    public Pose2d robotPos = new Pose2d(0, 0, 0);
    public Pose2d expectedPos=robotPos;
    public Trajectory t1;
    double x, y, turn;
    double xf, yf, turnf;

    SampleMecanumDrive drive;
    public ChassisSubsystem(final HardwareMap hMap) {
        register();
        drive = new SampleMecanumDrive(hMap);
    }

    public void moveTo(Pose2d targetPos1){
        expectedPos=targetPos1;
        t1 = drive.trajectoryBuilder(robotPos)
                .splineToSplineHeading(targetPos1, Math.toRadians(0))
                .build();
        drive.followTrajectoryAsync(t1);
    }
    public void moveTo(Pose2d targetPos1, Pose2d targetPos2){
        expectedPos=targetPos2;
        Trajectory t1 = drive.trajectoryBuilder(robotPos)
                .splineToSplineHeading(targetPos1, Math.toRadians(0))
                .splineToSplineHeading(targetPos2, Math.toRadians(0))
                .build();
        drive.followTrajectoryAsync(t1);
    }
    public boolean atCorrectPosition(){
        if((getX()<(egetX()+0.7)) && (getX()>(egetX()-0.7)) && (getY()>(egetY()-0.7)) && (getY()<(egetY()+0.7)) && (getHeading()>(egetHeading()-0.7))&& (getHeading()<(egetHeading()+0.7))){
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
    public void setTelePower(double xi, double yi, double turni){
        x=xi;
        y=yi;
        turn=turni;
    }
    public void TeleOp(){
        xf=-(x*Math.cos(-getHeading())-y*Math.sin(-getHeading()));
        yf=-(x*Math.sin(-getHeading())+y*Math.cos(-getHeading()));
        drive.setWeightedDrivePower(
                new Pose2d(
                        yf,
                        xf,
                        -turn
                )
        );

    }
    @Override
    public void periodic(){
        drive.update();
        robotPos = drive.getPoseEstimate();
    }
}