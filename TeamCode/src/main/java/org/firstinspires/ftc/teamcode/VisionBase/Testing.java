package org.firstinspires.ftc.teamcode.VisionBase;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.SubConstants;

@TeleOp
public class Testing extends LinearOpMode{
    DcMotorEx hSlide, arm, turntable;
    Servo dropper;
    Servo aligner;
    Servo deposit, grabber, grotate;
    AnalogInput Pot;
    DigitalChannel ttSensor;
    PIDCoefficients coefficients = new PIDCoefficients(0.02, 0, 0);
    BasicPID controller = new BasicPID(coefficients);
    ElapsedTime timer = new ElapsedTime();
    double armF = 0.18;
    @Override
    public void runOpMode() {
        hSlide = hardwareMap.get(DcMotorEx.class, "hslide");
        hSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlide.setDirection(DcMotorEx.Direction.REVERSE);
        dropper = hardwareMap.get(Servo.class, "dropper");
        aligner = hardwareMap.get(Servo.class, "aligner");
        deposit = hardwareMap.get(Servo.class, "aligner");
        Pot = hardwareMap.get(AnalogInput.class, "armpot");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        grabber = hardwareMap.get(Servo.class, "grabber");
        grotate = hardwareMap.get(Servo.class, "grotate");
        turntable = hardwareMap.get(DcMotorEx.class, "turntable");
        ttSensor = hardwareMap.get(DigitalChannel.class, "turntable");
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
        while (!isStopRequested()) {
            angle = (turntable.getCurrentPosition()/SubConstants.ticksperdeg);
            telemetry.addData("angle", angle);
            output = controller.calculate(45, angle);
            turntable.setPower(output);
            telemetry.addData("velocity", turntable.getVelocity());
            telemetry.addData("output", output);
            telemetry.update();
//while(ttSensor.getState())
//turntable.setPower(0.2);
        }
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
//            telemetry.addData("Slide position", vSlide.getCurrentPosition());
//            output = controller.calculate(900, vSlide.getCurrentPosition());
//            vSlide.setPower(output+0.1);
//            telemetry.addData("output", output);
//            telemetry.update();
//        }

}
