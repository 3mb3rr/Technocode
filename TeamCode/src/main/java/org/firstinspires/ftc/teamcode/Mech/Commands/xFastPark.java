package org.firstinspires.ftc.teamcode.Mech.Commands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class xFastPark extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;
    private final int zone;
    boolean temp1 = false;
    boolean temp2 = false;

    public xFastPark(ChassisSubsystem subsystem, int parkZone) {
        ChassisSub = subsystem;
        zone = parkZone;
        addRequirements(ChassisSub);

    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.parking;
        if ((ChassisSub.trajectoryCompleted) && (!temp1)) {
            if (ChassisSub.BLorRR) {
                if ((zone == 2) || (zone == 0))
                    ChassisSub.moveTo(new Pose2d(-49, 70.5, Math.toRadians(0)));
                else if (zone == 1)
                    ChassisSub.moveTo(new Pose2d(-49, 46.5, Math.toRadians(0)));
                else if (zone == 3)
                    ChassisSub.moveTo(new Pose2d(-49, 94.5, Math.toRadians(0)));
            } else {
                if ((zone == 2) || (zone == 0))
                    ChassisSub.moveTo(new Pose2d(-49, 70.5, Math.toRadians(0)));
                else if (zone == 1)
                    ChassisSub.moveTo(new Pose2d(-49, -25, Math.toRadians(0)));
                else if (zone == 3)
                    ChassisSub.moveTo(new Pose2d(-49, 23, Math.toRadians(0)));
            }
        }
    }

    @Override
    public void execute() {

        if(!(ChassisSub.drive.isBusy()))
        {
            ChassisSub.chassisState = ChassisSubsystem.chassis.parked;
        }

    }

    @Override
    public void end(boolean interrupted) {
        ChassisSub.chassisState = ChassisSubsystem.chassis.parked;
    }


    @Override
    public boolean isFinished() {
        return ChassisSub.atCorrectPosition();
    }

}