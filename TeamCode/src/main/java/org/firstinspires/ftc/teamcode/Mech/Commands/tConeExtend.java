package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDown;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;

import java.util.function.BooleanSupplier;

public class tConeExtend extends SequentialCommandGroup {

    public tConeExtend(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub)
    {
        BooleanSupplier hasCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return IntakeSub.hasCone();
            }
        };
        addCommands (
                new ConditionalCommand(
                    new InstantCommand(() -> {hSlideSub.hSlideSetPower(0.75);}),
                    new ParallelCommandGroup(new InstantCommand(() -> { hSlideSub.hSlideSetPower(0.75);}), new tArmDown(IntakeSub)),
                    hasCone)
                );
        addRequirements(IntakeSub);
    }

}

