package org.firstinspires.ftc.teamcode.VisionBase;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttCamera;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperMid;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttCamera;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.Camera2;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

@TeleOp
@Disabled
public class PoleTEst extends LinearOpMode {

    public Servo grotate;
    public boolean running = true;

    public void runOpMode() {

        Camera2 camera = new Camera2(hardwareMap);
        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;

        CommandScheduler.getInstance().registerSubsystem(camera, DepositSub);
        CommandScheduler.getInstance().schedule(new dropperMid(DepositSub));
        while((!isStopRequested()) && (!isStarted())){
            CommandScheduler.getInstance().run();
            telemetry.addLine("initialization");
            telemetry.addData("distance", camera.getPoleError());
            telemetry.update();
        }
CommandScheduler.getInstance().schedule(new ttCamera(DepositSub, camera));
while(!isStopRequested()){
    CommandScheduler.getInstance().run();
    telemetry.addData("distance", camera.getPoleError());
    telemetry.update();
}
    }
}
