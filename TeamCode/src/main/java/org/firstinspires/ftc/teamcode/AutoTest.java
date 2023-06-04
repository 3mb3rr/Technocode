package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

@Autonomous
public class AutoTest extends LinearOpMode {

    public Servo grotate;

    public void runOpMode() {

        grotate = hardwareMap.get(Servo.class, "grotate");
        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);
        hSlideSubsystem hSlideSub = new hSlideSubsystem(hardwareMap);
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;
        waitForStart();
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new AutoConeGrab(IntakeSub, hSlideSub),
            new AutoConeGrab(IntakeSub, hSlideSub),
            new AutoConeGrab(IntakeSub, hSlideSub),
            new AutoConeGrab(IntakeSub, hSlideSub),
            new AutoConeGrab(IntakeSub, hSlideSub)
        )
        );
        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("armAngle", IntakeSub.getArmAngle());
            telemetry.addData("hSlide", hSlideSub.getSlidePosition());
            telemetry.update();
        }
//while(!isStopRequested()){
//    vSlideSub.vSlideToPosition(300);
//}
    }
}
