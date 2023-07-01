package org.firstinspires.ftc.teamcode.Mech.Commands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.VisionBase.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class zoneDetection extends CommandBase {

    // The subsystem the command runs on
    private final Camera camera;


    // Tag ID 1,2,3 from the 36h11 family
    int LEFT= 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;

    public zoneDetection(Camera subsystem) {
        camera = subsystem;
        addRequirements(camera);
    }

    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        ArrayList<AprilTagDetection> currentDetections = Camera.aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    camera.inSight = tagFound;
                    break;
                }
            }

            if(tagFound)
            {
                camera.parkingZone = tagOfInterest.id;
            }

        }
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        return false;
    }

}