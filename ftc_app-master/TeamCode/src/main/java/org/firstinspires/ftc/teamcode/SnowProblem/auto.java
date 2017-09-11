package org.firstinspires.ftc.teamcode.SnowProblem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

/**
 * Created by Crystal Huynh on 9/10/2017.
 */

@Autonomous(name = "auto", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class auto{

    HardwareRIAW    robot       = new HardwareRIAW();              // Use a K9'shardware
    private ElapsedTime runtime = new ElapsedTime();
    Orientation angleRead = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    public enum State {TURNING, TURNED, FORWARD}

    public void move(double power){
        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(power);
    }

    public void turn(double power){
        robot.frontLeftDrive.setPower(-power);
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(-power);
        robot.backRightDrive.setPower(power);
    }

    public State gyroTurn (boolean left, double angle, double power){
        double initAngle = angleRead.firstAngle;
        double currAngle = angleRead.firstAngle;
        if (left && ((abs(currAngle - initAngle)) < angle)){
            turn(power);
            return State.TURNING;
        }
        else if (!left && ((abs(currAngle + initAngle)) < angle)){
            turn(-power);
            return State.TURNING;
        }
        turn(0);
        return State.TURNED;
    }
}
