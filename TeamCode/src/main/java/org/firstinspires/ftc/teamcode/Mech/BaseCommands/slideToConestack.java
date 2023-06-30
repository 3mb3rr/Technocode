package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class slideToConestack extends CommandBase {

    // The subsystem the command runs on
    private final hSlideSubsystem hSlideSub;
    private final IntakeSubsystem IntakeSub;

    public slideToConestack(hSlideSubsystem subsystem, IntakeSubsystem subsystem2) {
        hSlideSub = subsystem;
        IntakeSub = subsystem2;
        addRequirements(hSlideSub);
    }

    @Override
    public void initialize() {
        hSlideSub.hSlideState = hSlideSubsystem.HSlide.extending;
        hSlideSub.hSlideSetPower(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        hSlideSub.hSlideToPosition((int)hSlideSub.getSlidePosition());
        hSlideSub.hSlideState = hSlideSubsystem.HSlide.holding;
    }

    @Override
    public boolean isFinished() {
        if((hSlideSub.slideCurrentSpike()) || (IntakeSub.hasCone()) || ((hSlideSub.getSlideVelocity()<0.2) && (hSlideSub.getSlidePosition()>430)))
        {return true;}
        return false;
    }

}