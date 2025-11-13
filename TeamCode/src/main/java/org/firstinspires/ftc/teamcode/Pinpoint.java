package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Pinpoint {

    private final Telemetry telemetry;
    private final Chassis chassis;

    public GoBildaPinpointDriver odo;

    public Pinpoint(HardwareMap hw, Chassis ch, Telemetry tele, double offsetX, double offsetY) {

        chassis = ch;
        telemetry = tele;

        odo = hw.get(GoBildaPinpointDriver.class,"odo");
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(offsetX*25.4, offsetY*25.4, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        // odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
        //                          GoBildaPinpointDriver.EncoderDirection.FORWARD);
        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
//        odo.recalibrateIMU();
        odo.update();
        odo.resetPosAndIMU();
    }

    public void setEncoderDirection(GoBildaPinpointDriver.EncoderDirection xEncoder,
                                    GoBildaPinpointDriver.EncoderDirection yEncoder) {
        odo.setEncoderDirections(xEncoder, yEncoder);
    }

    @SuppressLint("DefaultLocale")
    public void moveForward(double dist, double power) throws InterruptedException {
        double dX;
        double motorPower = power*Math.signum(dist);
//        double kP_drive = 0.05;

        odo.resetPosAndIMU();
        telemetry.addLine(String.format("Move forward %.1f inches",dist));
        telemetry.addData("Power", "%.2f", motorPower);
        telemetry.addData("Initial X", "%.2f", odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Initial Y", "%.2f", odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Initial Heading (deg)", "%.1f", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Status",odo.getDeviceStatus());
        telemetry.update();

        chassis.frontLeftDrive.setPower(motorPower);
        chassis.backLeftDrive.setPower(motorPower);
        chassis.frontRightDrive.setPower(motorPower);
        chassis.backRightDrive.setPower(motorPower);

        do {
            odo.update();

            double currentX = odo.getPosX(DistanceUnit.INCH);

            dX = dist - currentX;
            telemetry.addData("target",dist);
            telemetry.addData("currentX",currentX);
            telemetry.addData("dX",dX);
            telemetry.update();
            sleep(50);
        } while (abs(dX)>1.);

        chassis.stopMotors();
        sleep(50);
    }

    @SuppressLint("DefaultLocale")
    public void moveSideways(double dist, double power) throws InterruptedException {
        double dY;
        double motorPower = power*Math.signum(dist);
//        double kP_drive = 0.05;

        odo.resetPosAndIMU();
        telemetry.addLine(String.format("Rotate %.1f inches",dist));
        telemetry.addData("Power", "%.2f", motorPower);
        telemetry.addData("Initial X", "%.2f", odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Initial Y", "%.2f", odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Initial Heading (deg)", "%.1f", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Status",odo.getDeviceStatus());
        telemetry.update();

        chassis.frontLeftDrive.setPower(motorPower);
        chassis.backLeftDrive.setPower(-motorPower);
        chassis.frontRightDrive.setPower(-motorPower);
        chassis.backRightDrive.setPower(motorPower);

        do {
            odo.update();

            double currentY = odo.getPosY(DistanceUnit.INCH);

            dY = dist + currentY;
            telemetry.addData("target",dist);
            telemetry.addData("currentY",currentY);
            telemetry.addData("dY",dY);
            telemetry.update();
//            sleep(1000);
//            ch.stopMotors();
//            sleep(100);
        } while (abs(dY)>1.);

        chassis.stopMotors();
        sleep(50);
    }
}
