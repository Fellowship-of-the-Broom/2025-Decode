package org.firstinspires.ftc.teamcode.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Launcher;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.crypto.spec.OAEPParameterSpec;

public class Robot {

    private final LinearOpMode opMode;
    private final Chassis chassis;

    private final AprilTag aprilTag;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Launcher launcher;
    private final Intake intake;



    public Robot(LinearOpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        aprilTag = new AprilTag (opMode);
        chassis = new Chassis (opMode, aprilTag);
        intake = new Intake(opMode);
        launcher =new Launcher(opMode);

    }
    public void init(){
        chassis.init();
        aprilTag.init();

    }
    public void start(){
//        launcher.start();
//        intake.start();
//       // aprilTag.start();
//        chassis.start():
        while (this.opMode.opModeIsActive()){
            intake.run();
            chassis.run();
            launcher.run();

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                //throw new RuntimeException(e);
            }
        }

    }

}
