package org.firstinspires.ftc.teamcode.SnowProblem;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="servoTester", group="RIAW")
//@Disabled
public class servoTester extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRIAW    robot           = new HardwareRIAW();              // Use a K9'shardware
    private ElapsedTime     runtime = new ElapsedTime();
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    double          gripperPosition = 0;
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
            runtime.reset();
            while(opModeIsActive() && runtime.seconds() < 3)
            {
                robot.gripper.setPosition(gripperPosition);
                telemetry.addData("servo value",   "%.2f", gripperPosition);
                telemetry.addData("runtime", runtime);
                telemetry.update();
            }
            runtime.reset();
            if (gripperPosition == 0.4) {
                gripperPosition = 0.6;
            }
            else
            {
                gripperPosition = 0.4;
            }
            // Send telemetry message to signify robot running;
            telemetry.addData("servo value",   "%.2f", gripperPosition);
            telemetry.addData("runtime", runtime);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}

