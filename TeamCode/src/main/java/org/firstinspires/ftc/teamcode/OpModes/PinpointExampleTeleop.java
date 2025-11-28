package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;

@TeleOp(name="PinpointExample-Teleop", group="Demo")
public class PinpointExampleTeleop extends OpMode {

    Chassis ch;

    Pinpoint pinpoint;

    @Override
    public void init() {

        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap, ch, telemetry, -5.5, 5.0, false);

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                     GoBildaPinpointDriver.EncoderDirection.REVERSED);
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

        ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Get current position
        double encoderX = pinpoint.odo.getEncoderX();
        double encoderY = pinpoint.odo.getEncoderY();
        double currentX = pinpoint.odo.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.odo.getPosY(DistanceUnit.INCH);
        double currentHeading = pinpoint.odo.getHeading(AngleUnit.DEGREES);

        telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
        telemetry.addLine(String.format("Encoder X: %6.2f", encoderX));
        telemetry.addLine(String.format("Encoder Y: %6.2f", encoderY));
        telemetry.addLine(String.format("Current X: %6.2f", currentX));
        telemetry.addLine(String.format("Current Y: %6.2f", currentY));
        telemetry.addLine(String.format("Current Heading: %.1f", currentHeading));
        telemetry.update();
    }
}