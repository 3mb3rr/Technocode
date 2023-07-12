package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class armDrop extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public armDrop(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }
    @Override
    public void initialize() {
        SubConstants.armFeedforward = 0.1;
        IntakeSub.armState = IntakeSubsystem.Arm.dropping;
        IntakeSub.grotateToAngle(76);
        IntakeSub.armToAngle(90);
    }
    @Override
    public void end(boolean interrupted) {
        IntakeSub.armState = IntakeSubsystem.Arm.holding;
        SubConstants.armFeedforward = 0.23;
    }
    @Override
    public boolean isFinished() {
        if((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()>87))
        {return true;}
        return false;
    }

}