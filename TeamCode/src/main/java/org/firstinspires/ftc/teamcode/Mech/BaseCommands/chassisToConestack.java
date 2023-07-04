package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class chassisToConestack extends CommandBase {

    // The subsystem the command runs on
    private final hSlideSubsystem hSlideSub;
    private final IntakeSubsystem IntakeSub;
    private final ChassisSubsystem ChassisSub;

    public chassisToConestack(hSlideSubsystem subsystem, IntakeSubsystem subsystem2, ChassisSubsystem subsystem3) {
        hSlideSub = subsystem;
        IntakeSub = subsystem2;
        ChassisSub = subsystem3;
        addRequirements(hSlideSub);
    }

    @Override
    public void initialize() {
        hSlideSub.hSlideState = hSlideSubsystem.HSlide.extending;
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
        ChassisSub.setWheelPowers(0.55, 0.55, 0.55, 0.55);
    }
    @Override
    public void execute(){
        ChassisSub.setWheelPowers(0.55, 0.55, 0.55, 0.55);
    }

    @Override
    public void end(boolean interrupted) {
        hSlideSub.hSlideToPosition((int)hSlideSub.getSlidePosition());
        hSlideSub.hSlideState = hSlideSubsystem.HSlide.holding;
        ChassisSub.brake();
    }

    @Override
    public boolean isFinished() {
        if((hSlideSub.slideCurrentSpike()) || (IntakeSub.hasCone()) || ((ChassisSub.getYVelocity()<0.2) && (ChassisSub.BLorRR) && (ChassisSub.getY()>-17)) || ((ChassisSub.getYVelocity()>-0.2) && (!ChassisSub.BLorRR) && (ChassisSub.getY()<17)))
        {return true;}
        return false;
    }

}