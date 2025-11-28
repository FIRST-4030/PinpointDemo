package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Pinpoint {

    private final Telemetry telemetry;
    private final Chassis chassis;

    public GoBildaPinpointDriver odo;

    Datalog datalog;
    boolean logData;

    double currentHeading, currentX, currentY;
    double targetHeading, targetX, targetY;
    double errorX, errorY, distanceToTarget;
    double error, power;

    public Pinpoint(HardwareMap hw, Chassis ch, Telemetry tele, double offsetX, double offsetY, boolean log) {

        chassis = ch;
        telemetry = tele;
        logData = log;

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
        odo.update();
        odo.resetPosAndIMU();

        // Initialize the datalog
        if (logData) {
            datalog = new Datalog("Pinpoint");
        }
    }

    public void setEncoderDirection(GoBildaPinpointDriver.EncoderDirection xEncoder,
                                    GoBildaPinpointDriver.EncoderDirection yEncoder) {
        odo.setEncoderDirections(xEncoder, yEncoder);
    }

    @SuppressLint("DefaultLocale")
    public void moveTo(double targetXa, double targetYa, double newRotation) {
        // Control parameters
        double kP = 0.15; // Proportional gain - tune this!
        double maxSpeed = 0.4; // Maximum drive speed
        double minSpeed = 0.1; // Maximum drive speed
        double tolerance = 0.5; // Position tolerance in inches

        targetX = targetXa;
        targetY = targetYa;

        while (true) {

            odo.update();

            // Get current position from Pinpoint
            currentX = odo.getPosition().getX(DistanceUnit.INCH);
            currentY = odo.getPosition().getY(DistanceUnit.INCH);

            // Calculate error
            errorX = targetX - currentX;
            errorY = targetY - currentY;

            // Calculate distance to target
            distanceToTarget = Math.hypot(errorX, errorY);

            if (distanceToTarget < tolerance) break;

            // Calculate velocities using proportional control
            double vX = errorX * kP;
            double vY = errorY * kP;

            // Limit to max speed
            double speed = Math.hypot(vX, vY);
            if (speed > maxSpeed) {
                vX = (vX / speed) * maxSpeed;
                vY = (vY / speed) * maxSpeed;
            } else if (speed > 0 && speed < minSpeed && distanceToTarget > tolerance) {
                // Scale up if too slow (but not at target)
                vX = vX / speed * minSpeed;
                vY = vY / speed * minSpeed;
            }

            // Drive field-relative (no rotation for this example)
            driveFieldRelative(vX, vY, newRotation);

            telemetry.addLine(String.format("Distance: %6.2f",distanceToTarget));
            telemetry.addLine(String.format("vX: %6.2f, vY: %6.2f",vX,vY));
            telemetry.update();

            if (logData) logOneSample();
        }

        chassis.stopMotors();
    }

    private void driveFieldRelative(double vx, double vy, double rotation) {
        // Get robot heading from Pinpoint's built-in IMU
        double heading = odo.getHeading(AngleUnit.RADIANS);

        // Rotate velocity vector by negative heading to convert to robot frame
        double robotVx = vx * Math.cos(-heading) - vy * Math.sin(-heading);
        double robotVy = vx * Math.sin(-heading) + vy * Math.cos(-heading);

        // Calculate motor powers for mecanum drive
        double frontLeftPower = robotVx + robotVy + rotation;
        double frontRightPower = robotVx - robotVy - rotation;
        double backLeftPower = robotVx - robotVy + rotation;
        double backRightPower = robotVx + robotVy - rotation;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                          Math.max(Math.abs(backLeftPower),           Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        chassis.frontLeftDrive.setPower(frontLeftPower);
        chassis.frontRightDrive.setPower(frontRightPower);
        chassis.backLeftDrive.setPower(backLeftPower);
        chassis.backRightDrive.setPower(backRightPower);
    }

    private void logOneSample() {
        datalog.targetX.set(targetX);
        datalog.targetY.set(targetY);
        datalog.currentX.set(currentX);
        datalog.currentY.set(currentY);
        datalog.errorX.set(errorX);
        datalog.errorY.set(errorY);
        datalog.distanceToTarget.set(distanceToTarget);
        datalog.writeLine();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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
        public Datalogger.GenericField targetX = new Datalogger.GenericField("Target-X");
        public Datalogger.GenericField targetY = new Datalogger.GenericField("Target-Y");
        public Datalogger.GenericField currentX = new Datalogger.GenericField("X");
        public Datalogger.GenericField currentY = new Datalogger.GenericField("Y");
        public Datalogger.GenericField errorX = new Datalogger.GenericField("Err X");
        public Datalogger.GenericField errorY = new Datalogger.GenericField("Err Y");
        public Datalogger.GenericField distanceToTarget = new Datalogger.GenericField("Distance");

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
                        targetX,
                        targetY,
                        currentX,
                        currentY,
                        distanceToTarget,
                        errorX,
                        errorY
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
