package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ttMidRight extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;

    public ttMidRight(DepositSubsystem subsystem) {
        DepositSub = subsystem;
        addRequirements(DepositSub);
    }

    @Override
    public void initialize() {
        DepositSub.turntableToAngle(SubConstants.ttRightAngle-10);
        DepositSub.ttState = DepositSub.ttState.turning;
    }
    @Override
    public void end(boolean interrupted) {
        DepositSub.ttState = DepositSub.ttState.holding;
    }

    @Override
    public boolean isFinished() {
        if((DepositSub.getTTVelocity()<2) && (DepositSub.getTTAngle()<(SubConstants.ttRightAngle-12.5)) && (DepositSub.getTTAngle()>(SubConstants.ttRightAngle-7.5)))
        {return true;}
        return false;
    }

}