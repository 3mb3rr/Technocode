package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ttCamera extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;
    private final Camera camera;

    public ttCamera(DepositSubsystem subsystem, Camera subsystem2) {
        DepositSub = subsystem;
        camera = subsystem2;
        addRequirements(DepositSub);
    }

    @Override
    public void initialize() {
        DepositSub.ttState = DepositSub.ttState.turning;
    }
    @Override
    public void execute() {
        if (camera.getPoleError()==1000){
            DepositSub.setTTPower(0.6);
        }
        else{
            DepositSub.ttCorrect(camera.getPoleError());
        }
    }
    @Override
    public void end(boolean interrupted) {
        DepositSub.ttState = DepositSub.ttState.holding;
    }

    @Override
    public boolean isFinished() {
        if((DepositSub.getTTVelocity()<2) && (camera.getPoleError()<5) && (camera.getPoleError()>-5))
        {return true;}
        return false;
    }

}