/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@Autonomous(name = "CleanClawsAutoRed", group = "Sensor")

public class CleanClawsAutoRed extends LinearOpMode {

    private Limelight3A limelight;
    double team = 1;

    double cX;
    double cY = 60;
    double cZ = 0;
    double cYaw = 0;
    double cPitch = 0;
    double cRoll = 0;

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

    double gTolerance = 0.2; // 0.1
    double dTolerance = 0.1;
    double changeToleranceSeconds = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        //leftExtender.setPower(0);
        //rightExtender.setPower(0);

        // Motor configurations
        frontLeft = hardwareMap.get(DcMotor.class, "front_left"); // negative
        frontRight = hardwareMap.get(DcMotor.class, "front_right"); // positive
        backLeft = hardwareMap.get(DcMotor.class, "back_left"); // negative
        backRight = hardwareMap.get(DcMotor.class, "back_right"); // positive

        arm       = hardwareMap.get(DcMotor.class, "arm");
        arm_wrist = hardwareMap.get(Servo.class,   "arm_wrist");
        arm_claw  = hardwareMap.get(Servo.class,   "arm_claw");
        arm_ext   = hardwareMap.get(DcMotor.class, "arm_ext");
        hz_wrist  = hardwareMap.get(Servo.class,   "hz_wrist");
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
        // moveInAngle(Math.PI/2,0.5,1000);

        waitForStart();

        boolean sigma = true;

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    // telemetry.addData("Botpose", botpose.toString());
                    cX = botpose.getPosition().x*39.37;
                    cY = botpose.getPosition().y*39.37;
                    cZ = botpose.getPosition().z*39.37;
                    cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                    telemetry.addData("Bot X", cX);
                    telemetry.addData("Bot Y", cY);


                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();

            // current auto
            while (sigma) {
                arm_claw.setPosition(0.78);
                arm.setTargetPosition(1300);
                arm.setPower(0.7);
                arm_wrist.setPosition(0.5);
                sleep(1000);

                frontLeft.setPower(0.8);
                frontRight.setPower(-0.8);
                backLeft.setPower(0.8);
                backRight.setPower(-0.8);
                sleep(120);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                // adjust y
                double tolerance = gTolerance;
                telemetry.addData("og og tolerance", tolerance);
                long t = System.currentTimeMillis();
                result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                        if (!(botpose.getPosition().x == 0 && botpose.getPosition().y == 0)) {
                            cX = botpose.getPosition().x*39.37;
                            cY = botpose.getPosition().y*39.37;
                            cZ = botpose.getPosition().z*39.37;
                        }
                        cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                        telemetry.addData("x", cX);
                        telemetry.addData("y", cY);
                        telemetry.addData("tolerance", tolerance);
                        sleep(50);
                        telemetry.update();
                    }
                }
                while (opModeIsActive() && (cY >= team*48.8+gTolerance || cY <= team*48.8-gTolerance)) {
                    if (cY <= team*48.8-gTolerance) {
                        frontLeft.setPower(-0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(0.25);
                    } else if (cY >= team*48.8+gTolerance) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(-0.25);
                        backLeft.setPower(0.25);
                        backRight.setPower(-0.25);
                    }
                    telemetry.addData("og tolerance", tolerance);
                    tolerance = gTolerance + (Math.floor((System.currentTimeMillis()-t)/(1000*changeToleranceSeconds)))*dTolerance;
                    telemetry.addData("t", t);
                    telemetry.addData("ctms minus t", System.currentTimeMillis()-t);
                    telemetry.addData("ctms sub t over s", (System.currentTimeMillis()-t)/(1000*changeToleranceSeconds));
                    telemetry.addData("final tolerance", tolerance);
                    result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            Pose3D botpose = result.getBotpose();
                            if (!(botpose.getPosition().x == 0 && botpose.getPosition().y == 0)) {
                                cX = botpose.getPosition().x*39.37;
                                cY = botpose.getPosition().y*39.37;
                                cZ = botpose.getPosition().z*39.37;
                            }
                            cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                            telemetry.addData("x", cX);
                            telemetry.addData("y", cY);
                            telemetry.addData("tolerance", tolerance);
                            sleep(50);
                            telemetry.update();
                        }
                    }
                }

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                sleep(500);

                arm.setTargetPosition(2100);
                arm.setPower(0.9);
                sleep(1000);
                arm_claw.setPosition(0.4);

                while (opModeIsActive() && (cYaw >= (Math.PI/2)+0.1 || cYaw <= (Math.PI/2)-0.1)) {
                    if (cYaw >= (Math.PI/2)+0.1) {
                        frontLeft.setPower(-0.25);
                        frontRight.setPower(-0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(-0.25);
                    } else if (cYaw <= (Math.PI/2)-0.1) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(0.25);
                        backRight.setPower(0.25);
                    }
                    telemetry.addData("og tolerance", tolerance);
                    tolerance = gTolerance + (Math.floor((System.currentTimeMillis()-t)/(1000*changeToleranceSeconds)))*dTolerance;
                    telemetry.addData("t", t);
                    telemetry.addData("ctms minus t", System.currentTimeMillis()-t);
                    telemetry.addData("ctms sub t over s", (System.currentTimeMillis()-t)/(1000*changeToleranceSeconds));
                    telemetry.addData("final tolerance", tolerance);
                    result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            Pose3D botpose = result.getBotpose();
                            if (!(botpose.getPosition().x == 0 && botpose.getPosition().y == 0)) {
                                cX = botpose.getPosition().x*39.37;
                                cY = botpose.getPosition().y*39.37;
                                cZ = botpose.getPosition().z*39.37;
                            }
                            cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                            telemetry.addData("x", cX);
                            telemetry.addData("y", cY);
                            telemetry.addData("tolerance", tolerance);
                            sleep(10);
                            telemetry.update();
                        }
                    }
                }
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(200);

                while (opModeIsActive() && (cX >= 0+gTolerance || cX <= 0-gTolerance)) {
                    if (cX >= 0+gTolerance) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(-0.25);
                    } else if (cX <= 0-gTolerance) {
                        frontLeft.setPower(-0.25);
                        frontRight.setPower(-0.25);
                        backLeft.setPower(0.25);
                        backRight.setPower(0.25);
                    }
                    telemetry.addData("og tolerance", tolerance);
                    tolerance = gTolerance + (Math.floor((System.currentTimeMillis()-t)/(1000*changeToleranceSeconds)))*dTolerance;
                    telemetry.addData("t", t);
                    telemetry.addData("ctms minus t", System.currentTimeMillis()-t);
                    telemetry.addData("ctms sub t over s", (System.currentTimeMillis()-t)/(1000*changeToleranceSeconds));
                    telemetry.addData("final tolerance", tolerance);
                    result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            Pose3D botpose = result.getBotpose();
                            if (!(botpose.getPosition().x == 0 && botpose.getPosition().y == 0)) {
                                cX = botpose.getPosition().x*39.37;
                                cY = botpose.getPosition().y*39.37;
                                cZ = botpose.getPosition().z*39.37;
                            }
                            cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                            telemetry.addData("x", cX);
                            telemetry.addData("y", cY);
                            telemetry.addData("tolerance", tolerance);
                            sleep(10);
                            telemetry.update();
                        }
                    }
                }


                while (opModeIsActive() && (cY >= team*50+gTolerance || cY <= team*50-gTolerance)) {
                    if (result != null) {
                        if (result.isValid()) {
                            Pose3D botpose = result.getBotpose();
                            if (!(botpose.getPosition().x == 0 && botpose.getPosition().y == 0)) {
                                cX = botpose.getPosition().x*39.37;
                                cY = botpose.getPosition().y*39.37;
                                cZ = botpose.getPosition().z*39.37;
                            }
                            cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                            telemetry.addData("x", cX);
                            telemetry.addData("y", cY);
                            telemetry.addData("tolerance", tolerance);
                            sleep(10);
                            telemetry.update();
                        }
                    }
                    if (cY <= team*50-gTolerance) {
                        frontLeft.setPower(-0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(0.25);
                    } else if (cY >= team*50+gTolerance) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(-0.25);
                        backLeft.setPower(0.25);
                        backRight.setPower(-0.25);
                    }
                    telemetry.addData("og tolerance", tolerance);
                    tolerance = gTolerance + (Math.floor((System.currentTimeMillis()-t)/(1000*changeToleranceSeconds)))*dTolerance;
                    telemetry.addData("t", t);
                    telemetry.addData("ctms minus t", System.currentTimeMillis()-t);
                    telemetry.addData("ctms sub t over s", (System.currentTimeMillis()-t)/(1000*changeToleranceSeconds));
                    telemetry.addData("final tolerance", tolerance);
                    result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            Pose3D botpose = result.getBotpose();
                            if (!(botpose.getPosition().x == 0 && botpose.getPosition().y == 0)) {
                                cX = botpose.getPosition().x*39.37;
                                cY = botpose.getPosition().y*39.37;
                                cZ = botpose.getPosition().z*39.37;
                            }
                            cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                            telemetry.addData("x", cX);
                            telemetry.addData("y", cY);
                            telemetry.addData("tolerance", tolerance);
                            sleep(10);
                            telemetry.update();
                        }
                    }
                }

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(100);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                backLeft.setPower(1);
                backRight.setPower(1);
                sleep(980);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                sleep(200);

                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
                sleep(300);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                hz_ext.setTargetPosition(-1600);
                hz_ext.setPower(0.5);
                sleep(1500);

                hz_wrist.setPosition(0.05);
                sleep(1500);

                hz_claw.setPosition(0);
                sleep(1500);
                hz_claw.setPosition(0.78);

                hz_ext.setTargetPosition(0);
                hz_ext.setPower(0.5);
                sleep(1500);

                arm_ext.setTargetPosition(2030);

                sigma = false;

                // END OF MAIN

            }

        }
        limelight.stop();
    }

    private void moveInAngle(double angle, double power, long time) {
        double startX = cX;
        double startY = cY;
        //angle = angle*2;
        // double dPosition = Math.sqrt((cX-startX)**2 + (cY-startY)**2);

        // while ()
        // while (Math.sqrt((cX-startX)*(cX-startX) + (cY-startY)*(cY-startY)) < dist) {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            Pose3D botpose = result.getBotpose();
            cX = botpose.getPosition().x*39.37;
            cY = botpose.getPosition().y*39.37;
            cZ = botpose.getPosition().z*39.37;
            cYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        }
        frontLeft.setPower(-power*Math.sin(angle+3*Math.PI/4));
        backRight.setPower(power*Math.sin(angle+3*Math.PI/4));
        frontRight.setPower(power*Math.sin(angle+Math.PI/4));
        backLeft.setPower(-power*Math.sin(angle+Math.PI/4));
        // }
        // frontLeft.setPower(-power*Math.sin(angle+3*Math.PI/4));
        // backRight.setPower(power*Math.sin(angle+3*Math.PI/4));
        // frontRight.setPower(power*Math.sin(angle+Math.PI/4));
        // backLeft.setPower(-power*Math.sin(angle+Math.PI/4));
        // // sleep(time);

        // 312 rpm at 1 power
        // circumference of wheel: 12.863 in
        // 66.8887128764 inches per second at 1 power
        // double time = Math.abs(1000*dist/(66.888712874*(frontRight.getPower()+backRight.getPower())/2));
        telemetry.addData("Angle:", angle);
        // telemetry.addData("Distance:", dist);
        telemetry.addData("Time:", time);
        telemetry.addData("Bot X:", cX);
        telemetry.addData("Bot Y:", cY);
        telemetry.addData("Bot Yaw:", cYaw);
        telemetry.addData("Bot X:", cX);
        telemetry.addData("Bot Y:", cY);
        telemetry.addData("Bot Yaw:", cYaw);

        telemetry.update();
        sleep(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

}                