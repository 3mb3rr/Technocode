package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;

public class grabberGrab extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public grabberGrab(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }

    @Override
    public void initialize() {
        IntakeSub.closeGrabber();
        IntakeSub.hasCone(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}