package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class highSlideOpen extends CommandBase {

    // The subsystem the command runs on
    private final vSlideSubsystem vSlideSub;

    public highSlideOpen(vSlideSubsystem subsystem) {
        vSlideSub = subsystem;
        addRequirements(vSlideSub);
    }

    @Override

    public void initialize() {
        vSlideSub.vSlideToPosition(SubConstants.hPolePos);
        vSlideSub.vSlideState = vSlideSubsystem.VSlide.extending;
    }

    @Override
    public void end(boolean interrupted) {
        vSlideSub.vSlideState = vSlideSubsystem.VSlide.holding;
    }

    public boolean isFinished() {
        if(((vSlideSub.getSlideVelocity()<0.5) && (vSlideSub.getSlidePosition()>500))||((vSlideSub.getSlideVelocity()<2) && (vSlideSub.getSlidePosition()<(SubConstants.hPolePos+2)) && (vSlideSub.getSlidePosition()>(SubConstants.hPolePos-2))))
        {return true;}
        return false;
    }

}