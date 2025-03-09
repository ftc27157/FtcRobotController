package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="cleanClawsTeleOp")
public class CleanClawsTeleOp extends LinearOpMode {

    /* Declare OpMode members */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    private DcMotor arm = null;
    private DcMotor arm_ext = null;
    private Servo arm_wrist = null;
    private Servo arm_claw = null;
    private DcMotor hz_ext = null;
    private Servo hz_wrist = null;
    private Servo hz_claw = null;

    final double ARM_TICKS_PER_DEGREE = (double) 194481 / 9826;
    final double HZ_WRIST_TICKS_PER_DEGREE = (double) 194481 / 9826;
    final double SUPPRESSANT = 1;
    boolean hzClawOpen = false;
    boolean armClawOpen = false;
    boolean armExtOpen = false;
    double armForward;
    boolean initialization = true;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left"); // negative
        frontRight = hardwareMap.get(DcMotor.class, "front_right"); // positive
        backLeft = hardwareMap.get(DcMotor.class, "back_left"); // negative
        backRight = hardwareMap.get(DcMotor.class, "back_right"); // positive

        arm       = hardwareMap.get(DcMotor.class, "arm");
        arm_wrist = hardwareMap.get(Servo.class,   "arm_wrist"); // Port 0
        arm_claw  = hardwareMap.get(Servo.class,   "arm_claw");
        arm_ext   = hardwareMap.get(DcMotor.class, "arm_ext");
        hz_wrist  = hardwareMap.get(Servo.class,   "hz_wrist"); // Port 0
        hz_claw   = hardwareMap.get(Servo.class,   "hz_claw");
        hz_ext    = hardwareMap.get(DcMotor.class, "hz_ext");

        // hz_ext.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        //arm_ext.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hz_ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        hz_ext.setTargetPosition(0);
        arm_ext.setTargetPosition(0);
        hz_ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hz_ext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_ext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while (opModeIsActive()) {
            if (initialization) {
                arm.setTargetPosition(30 * (int) ARM_TICKS_PER_DEGREE);
                arm.setPower(0.5);
                sleep(250);
                arm_wrist.setPosition(0.5);
                sleep(250);
                hz_wrist.setPosition(0.35);
                sleep(2500);
                arm.setTargetPosition(175);
                arm.setPower(0.5);
                initialization = false;
            }

            // Drive control logic
            double angle = 0;
            if (gamepad1.left_stick_x > 0 && -1*gamepad1.left_stick_y >= 0) {
                angle = Math.PI*3/2 + Math.atan(-1*gamepad1.left_stick_y/gamepad1.left_stick_x);
            }
            else if (gamepad1.left_stick_x <= 0 && -1*gamepad1.left_stick_y > 0) {
                angle = Math.atan(-1*gamepad1.left_stick_x/-1*gamepad1.left_stick_y);
            }
            else if (gamepad1.left_stick_x < 0 && -1*gamepad1.left_stick_y <= 0) {
                angle = Math.PI/2 + Math.atan(-1*gamepad1.left_stick_y/gamepad1.left_stick_x);
            }
            else if (gamepad1.left_stick_x >= 0 && -1*gamepad1.left_stick_y < 0) {
                angle = Math.PI + Math.atan(-1*gamepad1.left_stick_x/-1*gamepad1.left_stick_y);
            }

            double factor = SUPPRESSANT*(Math.sqrt(gamepad1.left_stick_x*gamepad1.left_stick_x+gamepad1.left_stick_y*gamepad1.left_stick_y));

            frontLeft.setPower(-factor*Math.sin(angle+3*Math.PI/4));
            backRight.setPower(factor*Math.sin(angle+3*Math.PI/4));
            frontRight.setPower(factor*Math.sin(angle+Math.PI/4));
            backLeft.setPower(-factor*Math.sin(angle+Math.PI/4));

            // Tank turn control logic
            double tank_turn_power = gamepad1.right_stick_x;
            frontRight.setPower(frontRight.getPower() - tank_turn_power);
            frontLeft.setPower(frontLeft.getPower() - tank_turn_power);
            backRight.setPower(backRight.getPower() - tank_turn_power);
            backLeft.setPower(backLeft.getPower() - tank_turn_power);

            // horizontal extender logic
            double hzExtPower = 0;
            if (gamepad1.dpad_up) {
                hzExtPower = SUPPRESSANT;
            } else if (gamepad1.dpad_down) {
                hzExtPower = -SUPPRESSANT;
            }
            if (gamepad2.dpad_up) {
                hzExtPower = SUPPRESSANT;
            } else if (gamepad2.dpad_down) {
                hzExtPower = -SUPPRESSANT;
            }
            int originalPos = hz_ext.getCurrentPosition();

            // Calculate target positions based on  input
            int finalPos = (int) (originalPos + hzExtPower * 100); // Adjust the multiplier (10) as needed

            // Set limits for extender positions
            if (finalPos < -1600) {
                finalPos = -1600;
            } else if (finalPos > 0) {
                finalPos = 0;
            }

            // Apply power to extenders and set target positions
            hz_ext.setTargetPosition(finalPos);

            if (Math.abs(hzExtPower) > 0.1) {
                hz_ext.setPower(0.9); // High power for movement
            } else {
                hz_ext.setPower(0.1); // Low power to hold position
            }

            // telemetry.addData("position", hz_ext.getCurrentPosition());
            // telemetry.addData("target", finalPos);
            // telemetry.addData("extender power", hzExtPower);
            // telemetry.update();

            // claws logic
            if (gamepad1.a){
                if (hzClawOpen) {
                    hz_claw.setPosition(0.41); // closed
                    hzClawOpen = false;
                    sleep(250);
                } else {
                    hz_claw.setPosition(0.15); // open
                    hzClawOpen = true;
                    sleep(250);
                }
            }

            if (gamepad1.x){
                if (armClawOpen) {
                    arm_claw.setPosition(0.78); // closed
                    armClawOpen = false;
                    sleep(250);
                } else {
                    arm_claw.setPosition(0.55); // open
                    armClawOpen = true;
                    sleep(250);
                }
            }

            // Vertical extenders logic
            if (gamepad1.y) {
                if (armExtOpen) {
                    arm_ext.setTargetPosition(0);
                    arm_ext.setPower(SUPPRESSANT);
                    armExtOpen = false;
                    sleep(250);
                } else {
                    arm_ext.setTargetPosition(2030);
                    arm_ext.setPower(SUPPRESSANT);
                    armExtOpen = true;
                    sleep(250);
                }
            }

            // arm logic
            armForward = - gamepad2.left_trigger + gamepad2.right_trigger;
            armForward = - gamepad1.left_trigger + gamepad1.right_trigger;
            int armOriginalPos = arm.getTargetPosition();
            int armFinalPos = (int) (armOriginalPos + armForward * 10);

            if (armFinalPos > 3500) {
                armFinalPos = 3500;
            } else if (armFinalPos < 0) {
                armFinalPos = 0;
            }

            arm.setTargetPosition(armFinalPos);

            if (Math.abs(armForward) > 0.1) {
                arm.setPower(0.9);
            } else {
                arm.setPower(0.2);
            }

            // parallel is hz wrist 0.2
            if (gamepad1.right_bumper) {
                hz_wrist.setPosition(0.35);
            }

            if (gamepad1.left_bumper) {
                hz_wrist.setPosition(0.05);
            }

            if (gamepad2.dpad_left) {
                hz_wrist.setPosition(hz_wrist.getPosition()-0.01);
            }

            if (gamepad2.dpad_right) {
                hz_wrist.setPosition(hz_wrist.getPosition()+0.01);
            }

            // if (gamepad2.x) {
            //     arm_wrist.setPosition(arm_wrist.getPosition()-0.01);
            // }

            // if (gamepad2.b) {
            //     arm_wrist.setPosition(arm_wrist.getPosition()+0.01);
            // }

            telemetry.addData("hz extender position", hz_ext.getCurrentPosition());
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("hz_wrist", hz_wrist.getPosition());
            telemetry.update();

            // Block transfer between claws
            // Arm position: 281
            // hz extender position: -1304
            // hz wrist: 0.4989
            // hzext=-1036, armpos=526,hz_wrist=0.7954
            // guess: armwrist 0.2
            if (gamepad1.b){
                arm_claw.setPosition(0.78);
                arm.setTargetPosition(1000); // move arm to make room for hz ext
                arm.setPower(0.5);
                sleep(1500);
                hz_wrist.setPosition(0.7954); // move wrist to correct position
                sleep(1000);
                hz_ext.setTargetPosition(-1136); // move ext to correct position
                hz_ext.setPower(0.9);
                sleep(1500);
                arm_wrist.setPosition(0.2); // correct arm wrist position
                sleep(1000);
                arm_claw.setPosition(0.40); // open arm claw
                sleep(1000);
                arm.setTargetPosition(546); // move arm to correct position
                sleep(1000);
                arm_claw.setPosition(0.78); // hold block with arm claw
                sleep(1000);
                // currently, both claws on block
                hz_claw.setPosition(0.15); // hz claw lets go of block
                sleep(1000);
                arm.setTargetPosition(1000); // move the block out of the way
                sleep(1500);
                hz_claw.setPosition(0.4);
                arm_wrist.setPosition(0.5); // reset wrists
                hz_wrist.setPosition(0.2);
                hz_claw.setPosition(0.4);
                armClawOpen = false; // match claw states with the variables
                hzClawOpen = true;
            }

            boolean isExpanded = false;
            if (gamepad2.a) {
                if (isExpanded) {
                    //contract
                    // isExpanded =
                } else {

                }
            }


            // telemetry.addData("position", arm.getCurrentPosition());
            // telemetry.update();
            // arm.setTargetPosition(30 * (int) ARM_TICKS_PER_DEGREE);
            // arm.setPower(0.5);

            // hz_ext.setTargetPosition(-500);
            // hz_ext.setPower(0.5);

            // hz_wrist.setPosition(0.17);
            // sleep(2000);
            // hz_claw.setPosition(0.2);
            // sleep(1000);
            // hz_claw.setPosition(1);
            // sleep(1000);
            // hz_wrist.setPosition(1);

            // sleep(3000);


            // arm.setTargetPosition(15 * (int) ARM_TICKS_PER_DEGREE);
            // arm.setPower(0.5);

            // hz_ext.setTargetPosition(-1000);
            // hz_ext.setPower(0.5);

        }
    }
}
