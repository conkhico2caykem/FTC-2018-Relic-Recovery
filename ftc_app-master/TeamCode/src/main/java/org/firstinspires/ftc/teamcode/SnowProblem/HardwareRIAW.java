package org.firstinspires.ftc.teamcode.SnowProblem;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

public class HardwareRIAW {
    /* Public OpMode members. */
    /* Left and Right determined with the robot facing forward */
    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    public Servo gripper = null;
    public Servo leftIntake = null;
    public Servo rightIntake = null;
    public Servo jewelArm = null;
    public BNO055IMU imu;

    public final static double ARM_HOME = 0.2;
    public final static double CLAW_HOME = 0.2;
    public final static double ARM_MIN_RANGE = 0.20;
    public final static double ARM_MAX_RANGE = 0.90;
    public final static double CLAW_MIN_RANGE = 0.20;
    public final static double CLAW_MAX_RANGE = 0.7;
    public double initAngle = 0;
    public double currAngle = 0;
    public double targetAngle = 0;
    public double clicksToInch = 0;
    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    Orientation angleRead;

    public enum State {READINIT, TURNING, TURNED, FORWARD}

    State state;

    /* Constructor */
    public HardwareRIAW() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive = hwMap.get(DcMotor.class, "fleft_drive");
        backLeftDrive = hwMap.get(DcMotor.class, "bleft_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "fright_drive");
        backRightDrive = hwMap.get(DcMotor.class, "bright_drive");
        //leftIntake = hwMap.get(DcMotor.class, "left_intake");
        //rightIntake = hwMap.get(DcMotor.class, "right_intake");
        //lift        = hwMap.get(DcMotor.class, "lift");
        //relicSlides = hwMap.get(DcMotor.class, "lift");
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        imu = hwMap.get(BNO055IMU.class, "imu");

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        //leftIntake.setPower(0);
        //rightIntake.setPower(0);
        //lift.setPower(0);
        //relicSlides.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        // Define and initialize ALL installed servos.
        gripper = hwMap.get(Servo.class, "gripper");
        //leftIntake       = hwMap.get(Servo.class, "left_intake");
        //rightIntake      = hwMap.get(Servo.class, "right_intake");
        //wrist            = hwMap.get(Servo.class, "wrist");
        //hand             = hwMap.get(Servo.class, "hand");
        //jewelArm         = hwMap.get(Servo.class, "jewel_arm");

        // State used for updating telemetry
        Orientation angles;
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);
    }

    public void move(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void turn(double power) {
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }

    public void intake(double power){

    }


    public void gyroTurn(boolean left, double angle, double power) {
        initAngle = angleRead.firstAngle;
        currAngle = imu.getAngularOrientation().firstAngle;

        if (left){
            if (((initAngle) + angle) > 180) {
                targetAngle = ((initAngle + angle) - 360);
            }
            else
            {
                targetAngle = initAngle + angle;
            }
        }
        else
            if (((initAngle - angle) < -180)) {
                targetAngle = ((initAngle - angle) + 360);
            }
            else{
                targetAngle = initAngle - angle;
        }

        if (left && (targetAngle > currAngle)) {
            state = State.TURNING;
            turn(power);
        } else if (!left && (targetAngle < currAngle)) {
            state = State.TURNING;
            turn(-power);
        } else {
            state = State.TURNED;
            power = 0;
            turn(power);
        }
    }

    public void encMove (boolean forward, double inches, double power)
    {
        clicksToInch = 10;   //converts encoder reading to inches
        double target = inches * clicksToInch;

        if (forward && abs(backLeftDrive.getCurrentPosition()) < target ) {
            move(power);
        }
        else if (!forward && abs(backLeftDrive.getCurrentPosition()) < target) {
            move(-power);
        }
        else{
            move(0);
        }
    }
}
