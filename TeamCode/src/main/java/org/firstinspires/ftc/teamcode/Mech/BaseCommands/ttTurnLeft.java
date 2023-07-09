package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ttTurnLeft extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;

    public ttTurnLeft(DepositSubsystem subsystem) {
        DepositSub = subsystem;
        addRequirements(DepositSub);
    }

    @Override
    public void initialize() {
        DepositSub.turntableToAngle(SubConstants.ttLeftAngle);
        DepositSub.ttState = DepositSub.ttState.turning;
    }
    @Override
    public void end(boolean interrupted) {
        DepositSub.ttState = DepositSub.ttState.holding;
    }

    @Override
    public boolean isFinished() {
        if((DepositSub.getTTVelocity()<2) && (DepositSub.getTTAngle()<(SubConstants.ttLeftAngle+2.5)) && (DepositSub.getTTAngle()>(SubConstants.ttLeftAngle-2.5)))
        {return true;}
        return false;
    }

}