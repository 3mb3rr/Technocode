package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class MediumSlideOpen extends CommandBase {

    // The subsystem the command runs on
    private final vSlideSubsystem vSlideSub;

    public MediumSlideOpen(vSlideSubsystem subsystem) {
        vSlideSub = subsystem;
        addRequirements(vSlideSub);
    }

    @Override
    public void initialize() {
        vSlideSub.vSlideToPosition(SubConstants.mPolePos);
        vSlideSub.vSlideState = vSlideSubsystem.VSlide.extending;
    }
    @Override
    public void end(boolean interrupted) {
        vSlideSub.vSlideState = vSlideSubsystem.VSlide.holding;
    }

    @Override
    public boolean isFinished() {
        if((vSlideSub.getSlideVelocity()<5) && (vSlideSub.getSlidePosition()<(SubConstants.hPolePos+5)) && (vSlideSub.getSlidePosition()>(SubConstants.hPolePos-5)))
        {return true;}
        return false;
    }

}