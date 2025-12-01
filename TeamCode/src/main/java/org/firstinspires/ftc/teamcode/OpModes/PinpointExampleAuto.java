package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;

@Autonomous(name="PinpointExample-Auto", group="Demo")
public class PinpointExampleAuto extends LinearOpMode {

    Chassis ch;
    Pinpoint pinpoint;
    Datalog datalog;
    int i = 1;
    double headingError;
    boolean localLog = false;
    double targetHeading, currentHeading;
    double distanceToTarget, kP_turn, turnSpeed, unWashedError;

    @Override
    public void runOpMode() {

        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap, ch, telemetry, -5.5, 5.0, true);

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                     GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // One calibration does not necessarily clear the hardware
        while ((abs(pinpoint.odo.getPosX(DistanceUnit.INCH))>0.1) &&
               (abs(pinpoint.odo.getPosX(DistanceUnit.INCH))>0.1)) {
            pinpoint.odo.update();
            pinpoint.odo.resetPosAndIMU();
            pinpoint.odo.recalibrateIMU();

//            telemetry.addData("Heading Scalar", pinpoint.odo.getYawScalar());
//            telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.INCH));
//            telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.INCH));
//            telemetry.addData("Initial Heading (deg)", "%.1f", pinpoint.odo.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Status",pinpoint.odo.getDeviceStatus());
//            telemetry.update();
        }

        pickOptions();

        // Initialize the datalog
        if (localLog) {
            datalog = new Datalog("AutoPinpoint");
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.DEGREES,0));
        pinpoint.odo.recalibrateIMU();
        sleep(500);
//        testHeadingControl(90);
//        sleep(500);
//        testHeadingControl(-90);
//        sleep(500);
//        testHeadingControl(90);

        pinpoint.moveTo(20,0, 45);
        sleep(500);
//        pinpoint.moveTo(0,0, 0);
////        pinpoint.moveTo(0,0, 0);
//        sleep(500);
//        pinpoint.moveTo(20,0, 0);
//        sleep(500);
//        pinpoint.moveTo(0,0, 0);
//        sleep(3000);
        stop();
//        pinpoint.moveForward(15, 0.4);
//        pinpoint.rotate(-90);
//        pinpoint.moveForward(-10, 0.4);
//        pinpoint.rotate(90);
//        pinpoint.moveSideways(10, 0.4);
//        pinpoint.rotate(90);
//        pinpoint.moveForward(-5, 0.4);
//        pinpoint.rotate(180);
//        pinpoint.moveSideways(5, 0.4);
//        pinpoint.rotate(90);
//        pinpoint.moveSideways(-10, 0.4);
    }

    // Test your heading controller in isolation
    public void testHeadingControl(double angle) {
        kP_turn = 0.1;  // Start here, adjust based on behavior
        targetHeading = Math.toRadians(angle);  // Want to face 90°
        double targetTolerance = Math.toRadians(3);
        telemetry.addData("i", i);
        telemetry.addData("kP_turn", kP_turn);
        telemetry.addData("Target Heading", Math.toDegrees(targetHeading));
//        telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
//        telemetry.addData("Error (deg)", Math.toDegrees(headingError));
//        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.update();
        i++;
        boolean done = false;

        while (opModeIsActive()) {

            pinpoint.odo.update();

            currentHeading = pinpoint.odo.getHeading(AngleUnit.RADIANS);

            // Calculate error
            headingError = targetHeading - currentHeading;
            unWashedError = headingError;

            // Wrap to [-π, π]
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            if (!done && abs(headingError) < targetTolerance) {
                done = true;
                headingError = 0;
//                break;
            }

            // Apply proportional control
            turnSpeed = Range.clip(headingError * kP_turn, -1.0, 1.0);

            // Only turn, no translation
            pinpoint.driveFieldRelative(0, 0, turnSpeed);

            telemetry.addData("kP_turn", kP_turn);
            telemetry.addData("Target Heading", Math.toDegrees(targetHeading));
            telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
            telemetry.addData("Error (deg)", Math.toDegrees(headingError));
            telemetry.addData("Turn Speed", turnSpeed);
            telemetry.addLine("Use gamepad to adjust kP_turn:");
            telemetry.addLine("DPad Up: increase, DPad Down: decrease");
            telemetry.update();

            // Allow tuning on the fly
            if (gamepad1.dpadUpWasReleased()) kP_turn += 0.05;
            if (gamepad1.dpadDownWasReleased()) kP_turn -= 0.05;
            kP_turn = Math.max(0.1, kP_turn);  // Don't go below 0.1

            if (localLog) { logOneSample(); }
        }

        ch.stopMotors();  // Stop
        sleep(3000);
    }
    @SuppressLint("DefaultLocale")
    void pickOptions() {

        int direction = 1;
        String directionText = "Up";
        double kP = pinpoint.getKp(), kPIncrement = 0.001;
        double kD = pinpoint.getKd(), kDIncrement = 0.00005;
        double kI = pinpoint.getKi(), kIIncrement = 0.001;

        double maxSpeed  = pinpoint.getMaxSpeed(),  maxSpeedIncrement  = 0.1;
        double tolerance = pinpoint.getTolerance(), toleranceIncrement = 0.1;

        boolean done = false;
        while (!done) {
            telemetry.addLine("Press & Release - X: kP,    Y: kI,    B: kD\n");
            telemetry.addLine("LB: Speed,  RB: Tolerance,  Back: Done\n");
            telemetry.addLine("DPAD Up: Increase,   DPAD Down: Down\n");
            telemetry.addLine(String.format("            Direction: %s\n",directionText));
            telemetry.addLine("           Current         Increment");
            telemetry.addLine(String.format("kP:        %5.4f             %5.4f",   kP, kPIncrement));
            telemetry.addLine(String.format("kI:         %4.3f              %5.3f",   kI, kIIncrement));
            telemetry.addLine(String.format("kD:        %6.5f             %6.5f\n", kD, kDIncrement));
            telemetry.addLine(String.format("Max Speed: %4.2f,    Tolerance:   %4.2f",maxSpeed,tolerance));
            telemetry.update();

            if (gamepad1.dpadUpWasReleased()) {
                direction =  1;
                directionText = "Up";
            }

            if (gamepad1.dpadDownWasReleased()) {
                direction = -1;
                directionText = "Down";
            }

            if (gamepad1.bWasReleased()) {
                kD = max(0, kD + direction * kDIncrement);
            }

            if (gamepad1.xWasReleased()) {
                kP = max(0, kP + direction * kPIncrement);
            }

            if (gamepad1.yWasReleased()) {
                kI = max(0, kI + direction * kIIncrement);
            }

            if (gamepad1.leftBumperWasReleased()) {
                maxSpeed = maxSpeed + direction * maxSpeedIncrement;
            }

            if (gamepad1.rightBumperWasReleased()) {
                tolerance = tolerance + direction * toleranceIncrement;
            }

            if (gamepad1.backWasReleased()) { done = true; }
        }

        pinpoint.setKp(kP);
        pinpoint.setKi(kI);
        pinpoint.setKd(kD);
        pinpoint.setMaxSpeed(maxSpeed);
        pinpoint.setTolerance(tolerance);

        ch.setMaxSpeed(maxSpeed);

        telemetry.addLine(String.format("kP: %5.4f,  kI: %5.4f,  kD: %6.5f\n",
                pinpoint.getKp(),pinpoint.getKi(),pinpoint.getKd()));
        telemetry.addLine(String.format("Max Speed: %4.2f",maxSpeed));
        telemetry.addLine(String.format("Tolerance:   %4.2f",tolerance));
        telemetry.update();
    }

    private void logOneSample() {
        datalog.targetHeading.set(Math.toDegrees(targetHeading));
        datalog.currentHeading.set(Math.toDegrees(currentHeading));
        datalog.headingError.set(Math.toDegrees(headingError));
        datalog.unWashedError.set(Math.toDegrees(unWashedError));
        datalog.kp_turn.set(kP_turn);
        datalog.turnSpeed.set(turnSpeed);
        datalog.writeLine();
    }

    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        /*
         * The underlying datalogger object - it cares only about an array of loggable fields
         */
        private final Datalogger datalogger;
        /*
         * These are all of the fields that we want in the datalog.
         * Note: Order here is NOT important. The order is important
         *       in the setFields() call below
         */
        public Datalogger.GenericField kp_turn = new Datalogger.GenericField("kPTurn");
        public Datalogger.GenericField targetHeading = new Datalogger.GenericField("Target H");
        public Datalogger.GenericField currentHeading = new Datalogger.GenericField("Current H");
        public Datalogger.GenericField headingError = new Datalogger.GenericField("Heading Err");
        public Datalogger.GenericField unWashedError = new Datalogger.GenericField("UnWashed Err");
        public Datalogger.GenericField turnSpeed = new Datalogger.GenericField("turnSpeed");

        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    /*
                     * Tell it about the fields we care to log.
                     * Note: Order *IS* important here! The order in which we list the
                     *       fields is the order in which they will appear in the log.
                     */
                    .setFields(
                            targetHeading,
                            currentHeading,
                            headingError,
                            unWashedError,
                            kp_turn,
                            turnSpeed
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}