package org.firstinspires.ftc.teamcode;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.SubConstants;

@TeleOp
public class Servotest extends LinearOpMode{
    DcMotorEx hSlide, arm;
    Servo dropper;
    Servo aligner;
    Servo deposit;
    AnalogInput Pot;
    PIDCoefficients coefficients = new PIDCoefficients(0.013, 0, 0);
    BasicPID controller = new BasicPID(coefficients);
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        hSlide = hardwareMap.get(DcMotorEx.class, "hslide");
        hSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlide.setDirection(DcMotorEx.Direction.REVERSE);
        dropper = hardwareMap.get(Servo.class, "dropper");
        aligner = hardwareMap.get(Servo.class, "aligner");
        deposit = hardwareMap.get(Servo.class, "deposit");
        Pot = hardwareMap.get(AnalogInput.class, "armpot");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        hSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double timeTaken = 0;
        double tpms = 1.91257;
        boolean idk = true;
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
            if (idk) {
                aligner.setPosition(0.3);
                dropper.setPosition(0.74);
                deposit.setPosition(0);
                idk = false;
            }
        }
//        while (!isStopRequested()) {
//            angle = (Pot.getVoltage()-0.584)/SubConstants.degpervolt;
//            telemetry.addData("encoder", angle);
//            telemetry.update();
//            output = controller.calculate(90, angle);
//            arm.setPower(output);
//            telemetry.addData("velocity", arm.getVelocity());
//            if((output<0.1) && (hSlide.getVelocity()<5) && (timeTaken == 0))  timeTaken = timer.milliseconds();
//            telemetry.addData("time taken", timeTaken);
//            telemetry.addData("output", output);
//            telemetry.update();
        }
//            telemetry.addData("Slide position", vSlide.getCurrentPosition());
//            output = controller.calculate(900, vSlide.getCurrentPosition());
//            vSlide.setPower(output+0.1);
//            telemetry.addData("output", output);
//            telemetry.update();
//        }
    }

