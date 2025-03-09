package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Dictionary;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp mode", group="Robot")
public class ConceptGoBildaStarterKitRobotTeleop_IntoTheDeep extends LinearOpMode {

    /* Declare OpMode members */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor leftExtender = null;
    public DcMotor rightExtender = null;
    public DcMotor arm = null;
    public Servo vertical = null;
    public Servo horizontal = null;
    public Servo pinion = null;

    // Constants for the arm's encoder calculations
    final double ARM_TICKS_PER_DEGREE = (double) 194481 / 9826;

    // Arm position setpoints
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    // Intake and wrist positions
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    // Pinion control
    final double PINION_CLOSED = 0.8;
    final double PINION_OPEN = 0.0;

    // Arm adjustment and suppress values
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    final double SUPPRESSANT = 1;

    // Variables for arm and drive
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    boolean pinionPerpendicularMode = false;
    String perpendicularOffModeReason = null;

    @Override
    public void runOpMode() {
        // Drive variables
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double armForward;
        double wristTurn;
        //double extenderRight;
        //double extenderLeft;

        // Initialize motors and servos
        frontLeft = hardwareMap.get(DcMotor.class, "front_left"); // negative
        frontRight = hardwareMap.get(DcMotor.class, "front_right"); // positive
        backLeft = hardwareMap.get(DcMotor.class, "back_left"); // negative
        backRight = hardwareMap.get(DcMotor.class, "back_right"); // positive
        leftExtender = hardwareMap.get(DcMotor.class, "left_extender"); // positive
        rightExtender = hardwareMap.get(DcMotor.class, "right_extender"); // negative
        arm = hardwareMap.get(DcMotor.class, "arm"); // negative
        vertical = hardwareMap.get(Servo.class, "vertical");
        horizontal = hardwareMap.get(Servo.class, "horizontal");
        pinion = hardwareMap.get(Servo.class, "pinion");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        //arm.setTargetPosition(2000);
        vertical.setPosition(0.5);
        horizontal.setPosition(0.5);
        pinion.setPosition(0);
        //leftExtender.setTargetPosition(0);
        //rightExtender.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtender.setTargetPosition(0);
        leftExtender.setTargetPosition(0);
        rightExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(0.25);

        //leftExtender.setPower(0);
        //rightExtender.setPower(0);

        waitForStart();

        // Motor configurations
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        while (opModeIsActive()) {
            // Drive control logic
            // frontRight.setPower((-1 * gamepad1.left_stick_y - gamepad1.left_stick_x) * SUPPRESSANT);
            // frontLeft.setPower(-(-1 * gamepad1.left_stick_y + gamepad1.left_stick_x) * SUPPRESSANT);
            // backRight.setPower((-1 * gamepad1.left_stick_y + gamepad1.left_stick_x) * SUPPRESSANT);
            // backLeft.setPower(-(-1 * gamepad1.left_stick_y - gamepad1.left_stick_x) * SUPPRESSANT);
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

            // Adjust arm motor position
            armForward = gamepad2.left_trigger - gamepad2.right_trigger;
            armForward = gamepad1.left_trigger - gamepad1.right_trigger;
            int originalPos = arm.getTargetPosition();
            int finalPos = (int) (originalPos + armForward * 40);
            double vertAngle = (double) (0.0534*originalPos + 312.57);
            if (vertAngle <= 140) {
                double angleNeeded = 180 - vertAngle;
                double newAngle = (angleNeeded + 30) / 240;
                vertical.setPosition(newAngle);
            } else if (140 < vertAngle && vertAngle <= 210) {
                double angleNeeded = 300 - vertAngle;
                double newAngle = (angleNeeded + 30) / 240;
                vertical.setPosition(newAngle);
            } else {
                vertical.setPosition(0.25);
            }
            telemetry.addData("final", finalPos);
            telemetry.addData("og", originalPos);
            telemetry.addData("forward", armForward);
            if (gamepad2.x) {
                double pinPos = (double) (1.5 * 1 - 0.75);
                pinion.setPosition(pinPos);
            } else if (gamepad2.y) {
                double pinPos = (double) (1.5 * 0.5 - 0.75);
                pinion.setPosition(pinPos);
            };
            if (gamepad1.x) {
                double pinPos = (double) (1.5 * 1 - 0.75);
                pinion.setPosition(pinPos);
            } else if (gamepad1.y) {
                double pinPos = (double) (1.5 * 0.5 - 0.75);
                pinion.setPosition(pinPos);
            };


            // Limit arm motor position
            if (finalPos <-4800) {
                finalPos = -4800;
            } else if (finalPos > 0) {
                finalPos = 0;
            }

            arm.setTargetPosition(finalPos);

            if (Math.abs(armForward) > 0.1) {
                arm.setPower(0.9);
            } else {
                arm.setPower(0.2);
            }
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double extenderPower = -gamepad2.left_stick_y * SUPPRESSANT; // Get power from left joystick Y-axis
            if (gamepad1.right_bumper) {
                extenderPower = SUPPRESSANT;
            } else if (gamepad1.left_bumper) {
                extenderPower = -SUPPRESSANT;
            }
            int originalPosLeft = leftExtender.getCurrentPosition();
            int originalPosRight = rightExtender.getCurrentPosition();
            
            // Calculate target positions based on joystick input
            int finalPosLeft = (int) (originalPosLeft + extenderPower * 100); // Adjust the multiplier (10) as needed
            int finalPosRight = (int) (originalPosRight - extenderPower * 100); // Invert power for the other motor
            
            // Set limits for extender positions
            if (finalPosLeft > 2000) {
                finalPosLeft = 2000;
            } else if (finalPosLeft < 0) {
                finalPosLeft = 0;
            }
            
            if (finalPosRight < -2000) {
                finalPosRight = -2000;
            } else if (finalPosRight > 0) {
                finalPosRight = 0;
            }
            
            // Apply power to extenders and set target positions
            leftExtender.setTargetPosition(finalPosLeft);
            rightExtender.setTargetPosition(finalPosRight);
            
            if (Math.abs(extenderPower) > 0.1) {
                leftExtender.setPower(0.9); // High power for movement
                rightExtender.setPower(0.9);
            } else {
                leftExtender.setPower(0.1); // Low power to hold position
                rightExtender.setPower(0.1);
            }
            
            // Add telemetry data for debugging
            telemetry.addData("Extender Power", extenderPower);
            telemetry.addData("Left Extender Position", originalPosLeft);
            telemetry.addData("Right Extender Position", originalPosRight);
            telemetry.addData("Final Left Position", finalPosLeft);
            telemetry.addData("Final Right Position", finalPosRight);
            telemetry.update();

            telemetry.update();
            // rightExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // leftExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Horizontal and vertical control logic
            if (gamepad2.dpad_left) {
                horizontal.setPosition(0.5);
            } else if (gamepad2.dpad_right) {
                horizontal.setPosition(0.1);
            } 
            double r = 0.02;
            if (gamepad1.dpad_left) {
                horizontal.setPosition(horizontal.getPosition()+r);
            }
            
            // if (gamepad1.dpad_right) {
            //     horizontal.setPosition(horizontal.getPosition()-r);
            // }
            
            if (gamepad1.a) {
                horizontal.setPosition(0.5);
            }
        }
    }

    public double armInDegrees(int ticks) {
        return ticks / ARM_TICKS_PER_DEGREE;
    }

    public int armInTicks(double degrees) {
        return (int) (degrees * ARM_TICKS_PER_DEGREE);
    }
}