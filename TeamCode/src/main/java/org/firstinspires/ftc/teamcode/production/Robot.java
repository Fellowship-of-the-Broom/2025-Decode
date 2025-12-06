package org.firstinspires.ftc.teamcode.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeIntakeImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeLauncherImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeTransferSystemImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.lib.mechanisms.IntakeImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.lib.mechanisms.LauncherImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.TransferSystem;
import org.firstinspires.ftc.teamcode.lib.mechanisms.TransferSystemImpl;

public class Robot {

    private final LinearOpMode opMode;
    private final Chassis chassis;

    private final AprilTag aprilTag;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Launcher launcher;
    private final Intake intake;
    private final TransferSystem transferSystem;
    private final AllianceColor allianceColor;


    public Robot(LinearOpMode opMode, boolean useReal, AllianceColor allianceColor){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        this.allianceColor = allianceColor;

        aprilTag = new AprilTag (opMode);
        chassis = new Chassis (opMode, aprilTag);
        if (useReal) {
            intake = new IntakeImpl(opMode);
            launcher = new LauncherImpl(opMode);
            transferSystem = new TransferSystemImpl(opMode);
        }
        else {
            intake = new FakeIntakeImpl();
            launcher = new FakeLauncherImpl();
            transferSystem = new FakeTransferSystemImpl();
        }



    }
    public void init(AllianceColor blueAlliance){
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
            transferSystem.run();

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                //throw new RuntimeException(e);
            }
        }

    }
    public void autoSleep(long timeMS){
        try {
            Thread.sleep(timeMS);
        } catch (InterruptedException e) {
            //throw new RuntimeException(e);
        }
    }
    public void rollout() {
        this.chassis.moveRobot(0, .5,0);
        autoSleep(1000);
        this.chassis.moveRobot(0,0,0);
    }

    public void autoMoveToAprilTagAndScore() {

        //Defaults to blue alliance values
        double strafeMultiplier = 1;
        double turnMultiplier = 1;

        if(allianceColor == AllianceColor.RED_ALLIANCE){
            strafeMultiplier = -1;
            turnMultiplier = -1;
        }
        //TODO Tune these values
        
        // Move forward to see april tag
        this.chassis.moveRobot(1, 0,turnMultiplier * 1);
        autoSleep(1000);
        this.chassis.moveRobot(0,0,0);

        // Turn to see april tag
        // Detect april tag & drive to it
        // Start launcher flywheel (and wait like ~3s)
        // (Open gate
        // Wait
        // Close gate
        // Wait ) x3
        // Stop flywheel
        // Possibly move back to start and/or out of the way
    }
}
