package org.firstinspires.ftc.teamcode.SnowProblem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by Crystal Huynh on 9/10/2017.
 */

@Autonomous(name = "gyroTest", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class gyroTest extends LinearOpMode {

    HardwareRIAW    robot           = new HardwareRIAW();              // Use a K9'shardware
    private ElapsedTime runtime = new ElapsedTime();
    Orientation angles;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        robot.angleRead = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        robot.initAngle = robot.imu.getAngularOrientation().firstAngle;
        while (opModeIsActive()) {
            //robot.angleRead = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            robot.gyroTurn(false, 30, .3);
            telemetry.addData("heading value",   "%.2f", robot.currAngle);
            telemetry.addData("intial heading",  "%.2f", robot.initAngle);
            telemetry.update();
        }
    }
}
