package org.firstinspires.ftc.teamcode.SnowProblem;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tank Drive", group="RIAW")
//@Disabled
public class teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRIAW    robot           = new HardwareRIAW();              // Use a K9'shardware
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.frontLeftDrive.setPower(left);
            robot.backLeftDrive.setPower(left);
            robot.frontRightDrive.setPower(right);
            robot.backRightDrive.setPower(right);

            if (gamepad1.left_bumper) {
                //intake
            }
            else if (gamepad1.left_trigger < 0.1) {
                //release
            }
            else {
                //motors off
            }

            if (gamepad1.dpad_up) {
                //move lift up
            }
            else if (gamepad1.dpad_down) {
                //move lift down
            }
            else{
                //stop lift
            }

            if (gamepad1.dpad_left) {
                //wrist left
            }
            else if (gamepad1.dpad_down) {
                //wrist right
            }
            else {
                //wrist stop
            }

            if (gamepad1.a) {
                //grab block
            }
            else if (gamepad1.b) {
                //release block
            }
            else {
                //stop hand
            }

            if (gamepad1.x) {
                //intake in
            }
            else if (gamepad1.y) {
                //intake out
            }
            else {
                //stop intake
            }

            // Send telemetry message to signify robot running;
            //telemetry.addData("arm",   "%.2f", armPosition);
            //telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}

