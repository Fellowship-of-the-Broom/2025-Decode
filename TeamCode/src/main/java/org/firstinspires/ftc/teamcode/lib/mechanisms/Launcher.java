package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher implements Runnable {
    private LinearOpMode opMode = null;
    private DcMotor flywheelMotor = null;
    private Servo hoodServo = null;
    private double power = 0;
    private static final double ANGLE_3FT = 0.5;
    private static final double manualIncrement = 20;
    public Launcher(LinearOpMode OpMode) {
        opMode = OpMode;

        this.flywheelMotor = opMode.hardwareMap.get(DcMotor.class, "flywheelMotor");
        this.hoodServo = opMode.hardwareMap.get(Servo.class, "hoodServo");

    }

//    public void start() {
//        Thread thread = new Thread(this);
//        thread.start();
//    }

    @Override
    public void run() {
       // while (this.opMode.opModeIsActive()) {

            // Flywheel motor
            // Not sure about the trigger and how this is going to be used on the gamepad
            this.flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            power = this.opMode.gamepad2.right_trigger;
            this.flywheelMotor.setPower(power);

            //Hood servo

            if (opMode.gamepad1.left_bumper || opMode.gamepad1.right_bumper) {
                hoodServo.setPosition(ANGLE_3FT);
            }

            if(opMode.gamepad2.left_stick_y != 0){
                double hoodAngle = hoodServo.getPosition() + (opMode.gamepad2.left_stick_y / manualIncrement);
                hoodAngle = Math.max(hoodAngle, 0);
                hoodAngle = Math.min(hoodAngle, 1);

                hoodServo.setPosition(hoodAngle);
            }

//            try {
//                Thread.sleep(100);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }


        }
    }
//}
