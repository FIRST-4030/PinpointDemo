package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;

@Autonomous(name="PinpointExample-Auto", group="Demo")
public class PinpointExampleAuto extends LinearOpMode {

    Chassis ch;
    Pinpoint pinpoint;

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

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.DEGREES,0));

        pinpoint.moveTo(10,5, 0);
        pinpoint.moveTo(10,25, 0);
        pinpoint.moveTo(0,0, 0);
        sleep(3000);
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
}