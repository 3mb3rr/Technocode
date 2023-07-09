package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class transfer extends SequentialCommandGroup {

    public transfer(IntakeSubsystem IntakeSub, DepositSubsystem DepositSub)
    {
        BooleanSupplier hasCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return IntakeSub.hasCone();
            }
        };
        addCommands (new SequentialCommandGroup(
                new armDrop(IntakeSub),
                new WaitCommand(280),
                new grabberOpen(IntakeSub),
                new InstantCommand(() -> {DepositSub.hasCone(true);})
                )
        );
        addRequirements(IntakeSub);
    }

}

