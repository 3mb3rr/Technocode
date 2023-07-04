package org.firstinspires.ftc.teamcode.Mech.Commands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class park extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;
    private final int zone;
    boolean temp1 = false;
    boolean temp2 = false;

    public park(ChassisSubsystem subsystem, int parkZone) {
        ChassisSub = subsystem;
        zone = parkZone;
        addRequirements(ChassisSub);

    }

    @Override
    public void initialize() {
        if ((ChassisSub.trajectoryCompleted) && (!temp1)) {
            if (ChassisSub.BLorRR) {
                if ((zone == 2) || (zone == 0))
                    ChassisSub.moveTo(new Pose2d(-50, 1, Math.toRadians(90)), -90);
                else if (zone == 1)
                    ChassisSub.moveTo(new Pose2d(-50, -23, Math.toRadians(90)), -90);
                else if (zone == 3)
                    ChassisSub.moveTo(new Pose2d(-50, 25, Math.toRadians(90)), -90);
            } else {
                if ((zone == 2) || (zone == 0))
                    ChassisSub.moveTo(new Pose2d(-50, -1, Math.toRadians(-90)), 90);
                else if (zone == 1)
                    ChassisSub.moveTo(new Pose2d(-50, -25, Math.toRadians(-90)), 90);
                else if (zone == 3) ChassisSub.moveTo(new Pose2d(-50, 23, Math.toRadians(-90)), 90);
            }
        }
    }

    @Override
    public void execute() {

        if((ChassisSub.trajectoryCompleted))
    {
        ChassisSub.chassisState = ChassisSubsystem.chassis.holding;
    }

    }

    @Override
    public void end(boolean interrupted) {
        ChassisSub.chassisState = ChassisSubsystem.chassis.holding;
    }


    @Override
    public boolean isFinished() {
        return ChassisSub.atCorrectPosition();
    }

}