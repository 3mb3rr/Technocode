package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class vSlideClose extends CommandBase {

    // The subsystem the command runs on
    private final vSlideSubsystem vSlideSub;

    public vSlideClose(vSlideSubsystem subsystem) {
        vSlideSub = subsystem;
        addRequirements(vSlideSub);
    }

    @Override
    public void initialize() {
        vSlideSub.vSlideState = vSlideSubsystem.VSlide.retracting;
        vSlideSub.setPower(-1);
    }
    @Override
    public void end(boolean interrupted) {
        vSlideSub.vSlideState = vSlideSubsystem.VSlide.holding;
        vSlideSub.setPower(0);
        vSlideSub.resetEncoder();
        vSlideSub.vSlideToPosition(0);
    }

    @Override
    public boolean isFinished() {
        if(((vSlideSub.getSlideVelocity()<5) && (vSlideSub.getSlidePosition()<(15))) || (vSlideSub.vClose()))
        {return true;}
        return false;
    }

}