package org.firstinspires.ftc.teamcode.Mech.subsystems;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class ChassisSubsystem extends SubsystemBase {

    public Pose2d robotPos;
    public Pose2d expectedPos=robotPos;
    SampleMecanumDrive drive;
    public ChassisSubsystem(final HardwareMap hMap) {
        drive = new SampleMecanumDrive(hMap);
    }

    public void moveTo(Pose2d targetPos1){
        Trajectory t1 = drive.trajectoryBuilder(robotPos)
                .splineToSplineHeading(targetPos1, Math.toRadians(0))
                .build();
        drive.followTrajectory(t1);
        expectedPos=targetPos1;
    }
    public void moveTo(Pose2d targetPos1, Pose2d targetPos2){
        Trajectory t1 = drive.trajectoryBuilder(robotPos)
                .splineToSplineHeading(targetPos1, Math.toRadians(0))
                .splineToSplineHeading(targetPos2, Math.toRadians(0))
                .build();
        drive.followTrajectory(t1);
        expectedPos=targetPos2;
    }
    public void moveTo(Pose2d targetPos1, Pose2d targetPos2, Pose2d targetPos3) {
        Trajectory t1 = drive.trajectoryBuilder(robotPos)
                .splineToSplineHeading(targetPos1, Math.toRadians(0))
                .splineToSplineHeading(targetPos2, Math.toRadians(0))
                .splineToSplineHeading(targetPos3, Math.toRadians(0))
                .build();
        expectedPos=targetPos3;
    }
    public boolean atCorrectPosition(){
        if(expectedPos.equals(robotPos))
            return true;
        return false;
    }
}