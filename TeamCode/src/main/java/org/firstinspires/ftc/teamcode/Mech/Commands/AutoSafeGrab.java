package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisToConestack;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.slideToConestack;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;

import java.util.function.BooleanSupplier;

public class AutoSafeGrab extends SequentialCommandGroup {

    InstantCommand DecreaseStackHeight = new InstantCommand(() ->{
        SubConstants.conestackHeight--;
    }) ;
    IntakeSubsystem IntakeSubs;

    public AutoSafeGrab(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub, ChassisSubsystem ChassisSub)
    {
        IntakeSubs=IntakeSub;
        BooleanSupplier DepConeDropped = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !IntakeSub.depositCone();
            }
        };
        BooleanSupplier IntakeConeDrop = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !IntakeSub.hasCone();
            }
        };
        BooleanSupplier SecondDrop = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return (!IntakeSub.hasCone() && IntakeSub.grabfailed);
            }
        };
        addCommands (
                new WaitUntilCommand(DepConeDropped),
                new chassisToConestack(hSlideSub, IntakeSub, ChassisSub),
                new grabberGrab(IntakeSub),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new armDrop(IntakeSub),
                        new hSlideClose(hSlideSub),
                        new chassisSafe(ChassisSub),
                        new InstantCommand(() ->{
                            SubConstants.conestackHeight--;
                        })
                ),
                new grabberOpen(IntakeSub),
                new WaitCommand(400),
                new InstantCommand(() -> {IntakeSub.botCommandComplete = true;})
        );

        addRequirements(IntakeSub);
    }

}