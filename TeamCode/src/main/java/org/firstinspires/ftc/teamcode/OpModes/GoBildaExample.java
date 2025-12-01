package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;

import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@TeleOp(name="GoBildaExample", group="Demo")

public class GoBildaExample extends LinearOpMode {

    Chassis ch;

    Pinpoint pinpoint;

    double oldTime = 0;
    double offsetX = -5.5;
    double offsetY = 5.0;

    @Override
    public void runOpMode() {

        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap, ch, telemetry, offsetX, offsetY, false);

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                     GoBildaPinpointDriver.EncoderDirection.REVERSED);

        //pinpoint.odo.recalibrateIMU();
        pinpoint.odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset", pinpoint.odo.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Device Version Number:", pinpoint.odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", pinpoint.odo.getYawScalar());
        telemetry.addData("Initial Heading", pinpoint.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Initial Heading from pose", pinpoint.odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            pinpoint.odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //pinpoint.odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);


            if (gamepad1.a){
                pinpoint.odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                pinpoint.odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = pinpoint.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("Heading", pinpoint.odo.getHeading(AngleUnit.DEGREES));

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", pinpoint.odo.getVelX(DistanceUnit.MM), pinpoint.odo.getVelY(DistanceUnit.MM), pinpoint.odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
            telemetry.addData("Status", pinpoint.odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", pinpoint.odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

        }
    }
}