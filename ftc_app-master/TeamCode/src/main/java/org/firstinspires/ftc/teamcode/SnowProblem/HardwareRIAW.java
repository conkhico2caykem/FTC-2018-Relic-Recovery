package org.firstinspires.ftc.teamcode.SnowProblem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareRIAW
{
    /* Public OpMode members. */
    /* Left and Right determined with the robot facing forward */
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backRightDrive   = null;
    public Servo    gripper          = null;
    public Servo    leftIntake       = null;
    public Servo    rightIntake      = null;
    public Servo    jewelArm         = null;

    public final static double ARM_HOME = 0.2;
    public final static double CLAW_HOME = 0.2;
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRIAW() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "fleft_drive");
        backLeftDrive   = hwMap.get(DcMotor.class, "bleft_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "fright_drive");
        backRightDrive  = hwMap.get(DcMotor.class, "bright_drive");
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        gripper          = hwMap.get(Servo.class, "gripper");
        //leftIntake       = hwMap.get(Servo.class, "left_intake");
        //rightIntake      = hwMap.get(Servo.class, "right_intake");
        //jewelArm         = hwMap.get(Servo.class, "jewel_arm");

        //.setPosition(ARM_HOME);
        //claw.setPosition(CLAW_HOME);
    }
}
