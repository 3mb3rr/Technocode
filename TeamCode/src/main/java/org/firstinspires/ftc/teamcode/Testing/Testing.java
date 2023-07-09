package org.firstinspires.ftc.teamcode.Testing;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;

@TeleOp
@Disabled
public class Testing extends LinearOpMode{
    DcMotorEx hSlide, arm, turntable, vSlide, leftFront, rightFront, leftRear, rightRear;
    Servo dropper;
    Servo aligner;
    Servo deposit, grabber, grotate;
    AnalogInput Pot;
    DistanceSensor gSensor, gSensor2;
    DigitalChannel ttSensor;
    DigitalChannel hClose, vClose;

    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.tKp, SubConstants.tKi, SubConstants.tKd);
    BasicPID controller = new BasicPID(coefficients);
    ElapsedTime timer = new ElapsedTime();

    @Override

    public void runOpMode() {
        hSlide = hardwareMap.get(DcMotorEx.class, "hslide");
        hSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide = hardwareMap.get(DcMotorEx.class, "vslide");
        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hSlide.setDirection(DcMotorEx.Direction.REVERSE);
        dropper = hardwareMap.get(Servo.class, "dropper");
        aligner = hardwareMap.get(Servo.class, "aligner");
        deposit = hardwareMap.get(Servo.class, "deposit");
        Pot = hardwareMap.get(AnalogInput.class, "armpot");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        grabber = hardwareMap.get(Servo.class, "grabber");
        grotate = hardwareMap.get(Servo.class, "grotate");
        turntable = hardwareMap.get(DcMotorEx.class, "turntable");
        ttSensor = hardwareMap.get(DigitalChannel.class, "turntable");
        gSensor = hardwareMap.get(DistanceSensor.class, "grabberSensor");
        gSensor2 = hardwareMap.get(DistanceSensor.class, "grabberSensor2");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);


        hSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double timeTaken = 0;
        double tpms = 1.91257;
        boolean idk = true;
        boolean idk2 = true;
        double angle = 0;
        int targetpos = 0;
        double output = 0;
        double ttOutput = 0;
        double ttAngle = 0;
        double armAngle = 0;
        double armOutput = 0;
        hClose = hardwareMap.get(DigitalChannel.class, "hclose");
        vClose = hardwareMap.get(DigitalChannel.class, "vclose");

        waitForStart();
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         double dropperPos = 1;
        while(!isStopRequested() && !isStarted()){
            telemetry.addData("dropper", dropperPos);
            dropper.setPosition(dropperPos);
            telemetry.update();
        }
        while(!isStopRequested()){
            telemetry.addData("dropper", dropperPos);
            dropperPos-=0.01;
            sleep(100);
            dropper.setPosition(dropperPos);
            telemetry.update();
        }
//        while(!isStopRequested()){
//            telemetry.addData("Slide position", vSlide.getCurrentPosition());
//            output = controller.calculate(850, vSlide.getCurrentPosition());
//            vSlide.setPower(output+0.1);
//            telemetry.addData("output", output);
//            telemetry.update();}
//        while (!isStopRequested()) {
//            if(idk) { timer.reset(); idk = false;}
//            telemetry.addData("Slide position", hSlide.getCurrentPosition());
//            output = controller.calculate(900, hSlide.getCurrentPosition());
//            hSlide.setPower(output);
//            telemetry.addData("velocity", hSlide.getVelocity());
//            if((output<0.1) && (hSlide.getVelocity()<5) && (timeTaken == 0))  timeTaken = timer.milliseconds();
//            telemetry.addData("time taken", timeTaken);
//            telemetry.addData("output", output);
//            telemetry.update();
//        }
//        while (!isStopRequested()) {
//            angle = (turntable.getCurrentPosition()*SubConstants.degspertick);
//            telemetry.addData("angle", angle);
//            output = controller.calculate(45, angle);
//            turntable.setPower(output);
//            telemetry.addData("velocity", turntable.getVelocity());
//            telemetry.addData("output", output);
//            telemetry.update();
////while(ttSensor.getState())
////turntable.setPower(0.2);
//        }
//        grotate.setPosition(0.59);
//        while (angle>-15) {
//            angle = (Pot.getVoltage()-0.584)/SubConstants.degpervolt;
//            telemetry.addData("angle", angle);
//            output = controller.calculate(-15, angle)+(armF*Math.cos(Math.toRadians(angle)));
//            arm.setPower(output);
//            telemetry.addData("velocity", arm.getVelocity());
//            telemetry.addData("output", output);
//            telemetry.update();}
//                grabber.setPosition(0.8);
//                grabber.setPosition(0.8);
//                sleep(500);
//                grabber.setPosition(0.57);
//                sleep(150);
//            grotate.setPosition(0.73);
//            while (angle<90) {
//            angle = (Pot.getVoltage()-0.584)/SubConstants.degpervolt;
//            telemetry.addData("angle", angle);
//            output = controller.calculate(90, angle)+(armF*Math.cos(Math.toRadians(angle)));
//            arm.setPower(output);
//            telemetry.addData("velocity", arm.getVelocity());
//            if((output<0.1) && (arm.getVelocity()<5) && (timeTaken == 0))  timeTaken = timer.milliseconds();
//            telemetry.addData("time taken", timeTaken);
//            telemetry.addData("output", output);
//            telemetry.update();}
//            if(angle>90) grabber.setPosition(0.8);

        }

//        }

}
