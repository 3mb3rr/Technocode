    package org.firstinspires.ftc.teamcode.Mech.subsystems;

    import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
    import com.arcrobotics.ftclib.command.SubsystemBase;
    import com.qualcomm.robotcore.hardware.AnalogInput;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.DistanceSensor;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.teamcode.Mech.SubConstants;

    public class IntakeSubsystem extends SubsystemBase {

        private final Servo grotate, grabber;
        private final AnalogInput Pot;
        public double armOutput = 0;
        public double armAngle = 0;
        public double grotatePos = 0;
        public int armTargetAngle = 90;
        public boolean level = false;
        public boolean depositCone = false;
        private final DcMotorEx arm;
        private final DistanceSensor gSensor;
        PIDCoefficients coefficients = new PIDCoefficients(SubConstants.aKp, SubConstants.aKi, SubConstants.aKd);
        BasicPID controller = new BasicPID(coefficients);
        public ElapsedTime slideTime = new ElapsedTime();
        public enum Grabber {
            hasCone, noCone
        }
        Grabber grabberState = Grabber.hasCone;
        public IntakeSubsystem(final HardwareMap hMap) {
            register();
            grotate = hMap.get(Servo.class, "grotate");
            grabber = hMap.get(Servo.class, "grabber");
            arm =  hMap.get(DcMotorEx.class, "arm");
            Pot = hMap.get(AnalogInput.class, "armpot");
            gSensor = hMap.get(DistanceSensor.class, "grabberSensor");
            armAngle = (Pot.getVoltage()-0.584)/SubConstants.degpervolt;
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            grotateToAngle(0);

        }



        public void openGrabber(){
            grabber.setPosition(SubConstants.grabberOpen);
        }
        public void closeGrabber(){
            grabber.setPosition(SubConstants.grabberClose);
        }
        public void grotateToAngle(int angle){
            level = false;
            grotatePos = 0.485+(angle*SubConstants.grotatePosPerDeg);
            grotate.setPosition(grotatePos);
        }
        public void grotateLevel(boolean value){
            level = value;
        }


        public double getArmVelocity() { return arm.getVelocity();}
        public double getArmAngle() { return armAngle;}
        public void armToAngle(int angle) {
                armTargetAngle = angle;
        }
        public void setArmPower(double power){
            armTargetAngle=1000; armOutput=power;
        }


        public boolean hasCone() {
            if(getDistance()<35) {
                grabberState = Grabber.hasCone;
            }
            else grabberState = Grabber.noCone;
            switch (grabberState) {
                case hasCone:
                    return true;
            }
            return false;
        }
        public double getDistance(){
            return gSensor.getDistance(DistanceUnit.CM);
        }
        public void hasCone(boolean decision){

            if(decision){grabberState = Grabber.hasCone;}
            else {grabberState = Grabber.noCone;}
        }
        public void depositCone(boolean decision){
            depositCone = decision;
        }

        public boolean depositCone(){
            return depositCone;
        }

        @Override
        public void periodic(){
            if (armTargetAngle!=1000){
            armAngle = (Pot.getVoltage()-0.584)/SubConstants.degpervolt;
            armOutput = controller.calculate(armTargetAngle, armAngle)+(SubConstants.armFeedforward*Math.cos(Math.toRadians(armAngle)));}
            arm.setPower(armOutput);
            if (level){
                grotatePos = 0.485+((-armAngle)*SubConstants.grotatePosPerDeg);
                grotate.setPosition(grotatePos);
            }


        }

    }