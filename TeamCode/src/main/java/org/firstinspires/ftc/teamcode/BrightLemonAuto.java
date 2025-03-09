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
@Autonomous(name = "BrightLemonAuto", group = "Sensor")

public class BrightLemonAuto extends LinearOpMode {

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
    public DcMotor leftExtender = null;
    public DcMotor rightExtender = null;
    public DcMotor arm = null;
    public Servo vertical = null;
    public Servo horizontal = null;
    public Servo pinion = null;

    double gTolerance = 0.1;
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
        leftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        leftExtender.setTargetPosition(10);
        rightExtender.setTargetPosition(-10);
        //arm.setTargetPosition(2000);
        horizontal.setPosition(0.5);
        pinion.setPosition(0.8);
        //leftExtender.setTargetPosition(0);
        //rightExtender.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtender.setPower(0.25);
        rightExtender.setPower(0.25);
        arm.setPower(0.25);

        //leftExtender.setPower(0);
        //rightExtender.setPower(0);

        // Motor configurations
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
                
                // move right to look at april tag
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                backLeft.setPower(1);
                backRight.setPower(1);
                sleep(319);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                // adjust y
                double tolerance = gTolerance;
                telemetry.addData("og og tolerance", tolerance);
                long t = System.currentTimeMillis();
                while (opModeIsActive() && (cY >= team*48+tolerance || cY <= team*48-tolerance)) {
                    if (cY <= team*48-tolerance) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(-0.25);
                    } else if (cY >= team*48+tolerance) {
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
                            //sleep(6000);
                            telemetry.update();
                        }
                    }
                }
                
                // adjust x
                tolerance = gTolerance;
                t = System.currentTimeMillis();
                while (opModeIsActive() && (cX >= team*55+tolerance || cX <= team*55-tolerance)) {
                    if (cX <= team*55-tolerance) {
                        frontLeft.setPower(-0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(0.25);
                    } else if (cX >= team*55+tolerance) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(-0.25);
                        backLeft.setPower(0.25);
                        backRight.setPower(-0.25);
                    }                   
                    telemetry.addData("og tolerance", tolerance);
                    tolerance = gTolerance + (Math.floor((System.currentTimeMillis()-t)/(1000*changeToleranceSeconds)))*dTolerance;                    result = limelight.getLatestResult();
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
                            telemetry.addData("gTolerance", gTolerance);
                            //sleep(6000);
                            telemetry.update();
                        }
                    }
                }
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                // turn left 45
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
                sleep(200);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                // move forward
                frontLeft.setPower(-1);
                frontRight.setPower(1);
                backLeft.setPower(-1);
                backRight.setPower(1);
                sleep(190);
                vertical.setPosition(0.696); 
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                // sleep(1000);
                
                // extenders fire
                leftExtender.setTargetPosition(1700);
                leftExtender.setPower(0.8);
                rightExtender.setTargetPosition(-1700);
                rightExtender.setPower(0.8);
                sleep(1500);
                leftExtender.setPower(0.1);
                rightExtender.setPower(0.1);
                
                // moves arm
                arm.setTargetPosition(-2650);
                arm.setPower(1);
                sleep(1500);
                
                
                // opens claws to release block
                pinion.setPosition(0);
                sleep(1000);
                arm.setTargetPosition(-500);
                sleep(1500);
                
                // moves extenders back down
                pinion.setPosition(0.5);
                leftExtender.setTargetPosition(10);
                rightExtender.setTargetPosition(-10);
                leftExtender.setPower(0.8);
                rightExtender.setPower(0.8);
                sleep(1500);
                leftExtender.setPower(0);
                rightExtender.setPower(0);
                
                // moves back to make space to turn
                frontLeft.setPower(1);
                frontRight.setPower(-1);
                backLeft.setPower(1);
                backRight.setPower(-1);
                sleep(220);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                
                // turns right 45 towards april tag
                vertical.setPosition(0.3);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(-1);
                sleep(220);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            
                sleep(500);    
                // adjust y
                tolerance = gTolerance;
                t = System.currentTimeMillis();
                while (opModeIsActive() && (cY >= team*48+tolerance || cY <= team*48-tolerance)) {
                    if (cY <= team*48-tolerance) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(-0.25);
                    } else if (cY >= team*48+tolerance) {
                        frontLeft.setPower(-0.25);
                        frontRight.setPower(-0.25);
                        backLeft.setPower(0.25);
                        backRight.setPower(0.25);
                    }
                    tolerance = gTolerance + (Math.floor((System.currentTimeMillis()-t)/(1000*changeToleranceSeconds)))*dTolerance;
                    result = limelight.getLatestResult();
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
                        telemetry.update();
                    }
                }
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                // adjust x
                tolerance = gTolerance;
                t = System.currentTimeMillis();
                while (opModeIsActive() && (cX >= team*56+tolerance || cX <= team*56-tolerance)) {
                    if (cX <= team*55-tolerance) {
                        frontLeft.setPower(-0.25);
                        frontRight.setPower(0.25);
                        backLeft.setPower(-0.25);
                        backRight.setPower(0.25);
                    } else if (cX >= team*55+tolerance) {
                        frontLeft.setPower(0.25);
                        frontRight.setPower(-0.25);
                        backLeft.setPower(0.25);
                        backRight.setPower(-0.25);
                    }
                    tolerance = gTolerance + (Math.floor((System.currentTimeMillis()-t)/(1000*changeToleranceSeconds)))*dTolerance;
                    result = limelight.getLatestResult();
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
                        telemetry.update();
                    }
                }
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500);
            
                // start pick block 
                // turn right to 90
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(-1);
                sleep(382);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500);
                
                // move forward
                frontLeft.setPower(-1);
                frontRight.setPower(1);
                backLeft.setPower(-1);
                backRight.setPower(1);
                sleep(40);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                vertical.setPosition(0.64); // make it parallel to ground at end // pause at top
                pinion.setPosition(0); // open intake
                sleep(500);
                arm.setTargetPosition(-4800); // bring arm completely down
                arm.setPower(1.0);
                sleep(2000);
                
                pinion.setPosition(0.8); // grab block
                sleep(500);
                
                arm.setTargetPosition(0); // bring arm back
                arm.setPower(0.8);
                sleep(2700);
                
                // Sleep
                sleep(100);
                
                // go back to apriltag position
                frontLeft.setPower(1);
                frontRight.setPower(-1);
                backLeft.setPower(1);
                backRight.setPower(-1);
                sleep(120);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                // turn left in front of basket
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
                sleep(500);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                // move forward
                frontLeft.setPower(-1);
                frontRight.setPower(1);
                backLeft.setPower(-1);
                backRight.setPower(1);
                sleep(150);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                vertical.setPosition(0.696);
                sleep(1000);
                
                // raise extenders
                leftExtender.setTargetPosition(1750);
                leftExtender.setPower(0.8);
                rightExtender.setTargetPosition(-1750);
                rightExtender.setPower(0.8);
                sleep(1500);
                leftExtender.setPower(0.1);
                rightExtender.setPower(0.1);
                
                // move arms
                arm.setTargetPosition(-2650);
                arm.setPower(1);
                sleep(1200);
                
                // drop block
                pinion.setPosition(0);
                sleep(1000);
                arm.setTargetPosition(0);
                sleep(1000);
                
                // bring extenders back down
                pinion.setPosition(0.5);
                leftExtender.setTargetPosition(0);
                rightExtender.setTargetPosition(0);
                leftExtender.setPower(1);
                rightExtender.setPower(1);
                sleep(1500);
                leftExtender.setPower(0.1);
                rightExtender.setPower(0.1);

                frontLeft.setPower(-1);
                frontRight.setPower(1);
                backLeft.setPower(-1);
                backRight.setPower(1);
                sleep(120);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                // // back to apriltag position
                // frontLeft.setPower(-1);
                // frontRight.setPower(-1);
                // backLeft.setPower(-1);
                // backRight.setPower(-1);
                // sleep(193);
                // frontLeft.setPower(0);
                // frontRight.setPower(0);
                // backLeft.setPower(0);
                // backRight.setPower(0);
                
                
                
                // moveInAngle(5*Math.PI/4, 0.1, 8000);
                
                sigma = false;
                
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
