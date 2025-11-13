package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;

@Autonomous(name="PinpointExample-Auto", group="Demo")
public class PinpointExampleAuto extends LinearOpMode {

    Chassis ch;
    Pinpoint pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap, ch, telemetry, -5.5, 5.0);

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                     GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // One calibration does not necessarily clear the hardware
        while ((abs(pinpoint.odo.getPosX(DistanceUnit.INCH))>0.1) &&
               (abs(pinpoint.odo.getPosX(DistanceUnit.INCH))>0.1)) {
            pinpoint.odo.update();
            pinpoint.odo.resetPosAndIMU();
            pinpoint.odo.recalibrateIMU();

            telemetry.addData("Heading Scalar", pinpoint.odo.getYawScalar());
            telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Initial Heading (deg)", "%.1f", pinpoint.odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status",pinpoint.odo.getDeviceStatus());
            telemetry.update();
        }

        telemetry.addLine("Calibrated");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        pinpoint.moveForward(10, 0.4);
        pinpoint.moveSideways(10, 0.4);
        pinpoint.moveForward(-15, 0.4);
        pinpoint.moveSideways(-10, 0.4);
        pinpoint.moveSideways(10, 0.4);
        pinpoint.moveForward(15, 0.4);
        pinpoint.moveSideways(-10, 0.4);
        pinpoint.moveForward(-10, 0.4);
//        moveTo(10, 5, 0, 0.5, true);
    }

    void moveTo(double targetX, double targetY, double targetHeading, double maxPower, boolean displayResults) {
        // The core of this code was derived from a ChatGPT query
        double kP_drive = 0.02;
        double kP_turn = 0.01;
        double headingError, distance;

        do {
            pinpoint.odo.update();

            double currentX = pinpoint.odo.getPosX(DistanceUnit.INCH);
            double currentY = pinpoint.odo.getPosY(DistanceUnit.INCH);
            double currentHeading = pinpoint.odo.getHeading(AngleUnit.RADIANS);

            double dx = targetX - currentX;
            double dy = targetY - currentY;
            distance = Math.hypot(dx, dy);
    //        double targetHeading = Math.atan2(dy, dx);
            headingError = Math.toRadians(targetHeading) - currentHeading;

            // Normalize heading error
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Compute control outputs
            double drive = kP_drive * distance;
            double turn = kP_turn * headingError;
            telemetry.addData("Current X", "%.2f", currentX);
            telemetry.addData("Current Y", "%.2f", currentY);
            telemetry.addData("Current Heading", "%.1f", Math.toDegrees(currentHeading));
            telemetry.addData("dX", dx);
            telemetry.addData("dY", dy);
            telemetry.addData("distance", distance);
            telemetry.addData("heading error", headingError);
            telemetry.addData("drive", drive);
            telemetry.addData("turn", turn);

            // Stop condition
    //        if (distance < 1.0) {
    //            drive = 0;
    //            turn = 0;
    //        }

            double fl = drive - turn;
            double fr = drive + turn;
            double bl = drive - turn;
            double br = drive + turn;

            // Normalize to prevent overpowering
            double max = Math.max(maxPower, abs(fl));
            max = Math.max(max, abs(fr));
            max = Math.max(max, abs(bl));
            max = Math.max(max, abs(br));
            telemetry.addData("max", max);
            telemetry.update();
//            sleep(3000);

            ch.frontLeftDrive.setPower(fl / max);
            ch.frontRightDrive.setPower(fr / max);
            ch.backLeftDrive.setPower(bl / max);
            ch.backRightDrive.setPower(br / max);

            if (displayResults) {
                telemetry.addData("Initial X", "%.2f", currentX);
                telemetry.addData("Initial Y", "%.2f", currentY);
                telemetry.addData("Initial Heading (deg)", "%.1f", Math.toDegrees(currentHeading));
                telemetry.addData("Distance to Target", "%.2f", distance);
                telemetry.update();
//                sleep(3000);
            }
        } while (distance > 1.0);

        ch.stopMotors();

        if (displayResults) {
            double finalX = pinpoint.odo.getPosX(DistanceUnit.INCH);
            double finalY = pinpoint.odo.getPosY(DistanceUnit.INCH);
            double finalHeading = pinpoint.odo.getHeading(AngleUnit.RADIANS);

            telemetry.addData("Final X", "%.2f", finalX);
            telemetry.addData("Final Y", "%.2f", finalY);
            telemetry.addData("Final Heading (deg)", "%.1f", Math.toDegrees(finalHeading));
            telemetry.update();
        }
//        sleep(5000);
    }
}