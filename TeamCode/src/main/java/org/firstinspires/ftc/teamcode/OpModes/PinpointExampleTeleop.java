package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;

@TeleOp(name="PinpointExample-Teleop", group="Demo")
public class PinpointExampleTeleop extends OpMode {

    Chassis ch;

    Pinpoint pinpoint;

    Datalog datalogTeleop;
    boolean localLog = true;
    double leftStickY = 0, leftStickX = 0, rightStickX = 0;

    @Override
    public void init() {

        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap, ch, telemetry, -5.5, 5.0, false);

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                     GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Initialize the datalogTeleop
        if (localLog) {
            datalogTeleop = new Datalog("TeleopPinpoint");
            localLog = false;  // Create the file in the possibility it will be written to
        }
    }

    @Override
    public void init_loop() {
        pinpoint.odo.update();
        pinpoint.odo.resetPosAndIMU();
        pinpoint.odo.recalibrateIMU();

        telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Initial Heading (deg)", "%.1f", pinpoint.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
        telemetry.update();
    }

    @Override
    public void start() {
        // Set values one more time when starting
        pinpoint.odo.resetPosAndIMU();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        if (gamepad1.leftBumperWasReleased()) {  // Start logging
            localLog = true;
        }

        if (gamepad1.rightBumperWasReleased()) {  // Stop logging
            localLog = false;
        }
        ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        leftStickY  = gamepad1.left_stick_y;
        leftStickX  = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        if (localLog) logOneSample();

        // Get current position
        double encoderX = pinpoint.odo.getEncoderX();
        double encoderY = pinpoint.odo.getEncoderY();
        double currentX = pinpoint.odo.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.odo.getPosY(DistanceUnit.INCH);
        double currentHeading = pinpoint.odo.getHeading(AngleUnit.DEGREES);

        telemetry.addLine("LB: Start logging,   RB: Stop logging\n");
        telemetry.addData("LeftStickY:",gamepad1.left_stick_y);
        telemetry.addData("leftStickY:",leftStickY);
        telemetry.addData("LeftStickX:",gamepad1.left_stick_x);
        telemetry.addData("RightStickX:",gamepad1.right_stick_x);
        telemetry.addLine("");
        telemetry.addLine(String.format("Encoder X: %6.2f", encoderX));
        telemetry.addLine(String.format("Encoder Y: %6.2f", encoderY));
        telemetry.addLine(String.format("Current X: %6.2f", currentX));
        telemetry.addLine(String.format("Current Y: %6.2f", currentY));
        telemetry.addLine(String.format("Current Heading: %.1f", currentHeading));
        telemetry.update();
    }

    private void logOneSample() {
        datalogTeleop.leftStickY.set(leftStickY);
        datalogTeleop.leftStickX.set(leftStickX);
        datalogTeleop.rightStickX.set(rightStickX);
        datalogTeleop.writeLine();
    }

    /**
     * Datalog class encapsulates all the fields that will go into the datalogTeleop.
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
        public Datalogger.GenericField leftStickY  = new Datalogger.GenericField("LeftY");
        public Datalogger.GenericField leftStickX  = new Datalogger.GenericField("LeftX");
        public Datalogger.GenericField rightStickX = new Datalogger.GenericField("RightX");

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
                            leftStickY,
                            leftStickX,
                            rightStickX
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