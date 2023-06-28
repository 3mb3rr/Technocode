package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class fallenConeGrab extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public fallenConeGrab(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }

    @Override
    public void initialize() {
            IntakeSub.armState = IntakeSubsystem.Arm.grabbing;
        IntakeSub.armToAngle(0);
        IntakeSub.grotateToAngle(-90);
    }
    @Override
    public void end(boolean interrupted) {
        IntakeSub.armState = IntakeSubsystem.Arm.holding;
    }

    @Override
    public boolean isFinished() {
            if(((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()<1.5) && (IntakeSub.getArmAngle()>-1.5))
                || (IntakeSub.getArmVelocity()==0))
        {return true;}
        return false;
    }

}