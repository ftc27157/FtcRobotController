package org.firstinspires.ftc.teamcode;// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;
// import org.openftc.easyopencv.OpenCvPipeline;
// import org.opencv.core.Core;
// import org.opencv.core.Mat;
// import org.opencv.core.Scalar;
// import org.opencv.core.Rect;
// import org.opencv.core.MatOfPoint;
// import org.opencv.imgproc.Imgproc;
// import java.util.ArrayList;
// import java.util.List;

// @Autonomous(name="FTC Basic Camera Stream", group="Robot")
// public class AutoMode extends LinearOpMode {

//     // Declare OpMode members
//     public DcMotor leftDrive = null;
//     public DcMotor rightDrive = null;
//     OpenCvCamera webcam;
//     ColorDetectionPipeline pipeline;

//     // Wheel rotation speed and diameter
//     private static final double MAX_MOTOR_POWER = 0.5; // Max power for motors
//     private static final int FRAME_LIMIT = 5; // Limit to process every 5 frames

//     @Override
//     public void runOpMode() {
//         // Initialize motors
//         leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
//         rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");

//         // Set motor modes to use velocity control
//         leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//         // Camera initialization
//         WebcamName webcamName = hardwareMap.get(WebcamName.class, "cam");
//         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                 "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

//         // Create OpenCV camera instance for webcam
//         webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//         pipeline = new ColorDetectionPipeline(this); // Pass the AutoMode instance

//         webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//             @Override
//             public void onOpened() {
//                 webcam.setPipeline(pipeline); // Set the pipeline for processing
//                 webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//                 telemetry.addLine("Camera streaming");
//                 telemetry.update();
//             }

//             @Override
//             public void onError(int errorCode) {
//                 telemetry.addLine("Camera error");
//                 telemetry.update();
//             }
//         });

//         telemetry.addLine("Robot Ready.");
//         telemetry.update();

//         // Wait for the game driver to press play
//         waitForStart();

//         // Spin until a bounding box is detected
//         while (opModeIsActive() && !pipeline.hasDetectedBoundingBox()) {
//             spinInPlace();
//             telemetry.addLine("Spinning... Waiting for bounding box detection.");
//             telemetry.update();
//         }

//         // Once a bounding box is detected, get the dimensions and move towards it
//         if (pipeline.hasDetectedBoundingBox()) {
//             telemetry.addLine("Bounding box detected!");
//             telemetry.update();
//             //moveToBoundingBox();
//         }

//         // Stop camera streaming after the game ends
//         webcam.stopStreaming();
//     }

//     // Spin in place
//     // private void spinInPlace() {
//     //     leftDrive.setPower(MAX_MOTOR_POWER); // Spin left motor forward
//     //     rightDrive.setPower(-MAX_MOTOR_POWER); // Spin right motor backward

//     //     // Continue spinning until a bounding box is detected or until the operation is inactive
//     //     while (opModeIsActive() && !pipeline.hasDetectedBoundingBox()) {
//     //         idle(); // Yield control to other tasks
//     //     }

//     //     // Stop the motors after detecting the bounding box or if the op mode is inactive
//     //     leftDrive.setPower(0);
//     //     rightDrive.setPower(0);
//     // }

//     // Move towards the detected bounding box
//     // private void moveToBoundingBox() {
//     //     double distance = pipeline.getLastDistance();
//     //     double angle = pipeline.getLastBoundingBoxAngle();

//     //     // Adjust motors based on the angle to turn towards the bounding box
//     //     double forward = -MAX_MOTOR_POWER; // Constant forward motion
//     //     double rotate = angle * 0.2; // Use angle to adjust turning power
//     //     double leftPower = forward + rotate; // Calculate left motor power
//     //     double rightPower = forward - rotate; // Calculate right motor power

//     //     // Set powers to drive towards the bounding box
//     //     leftDrive.setPower(leftPower);
//     //     rightDrive.setPower(rightPower);

//     //     // Move for a duration based on the detected distance
//     //     sleep((long) (distance * 1000)); // Move for a time proportional to the distance

//     //     // Stop the motors
//     //     leftDrive.setPower(0);
//     //     rightDrive.setPower(0);
//     // }

//     // Color detection pipeline class
//     public class ColorDetectionPipeline extends OpenCvPipeline {
//         // HSV thresholds for brighter yellow color detection
//         private final Scalar YELLOW_MIN = new Scalar(20, 100, 100); // Lowered H for broader detection
//         private final Scalar YELLOW_MAX = new Scalar(60, 255, 255); // Adjusted max values for broader detection

//         private Mat hsvImage = new Mat();
//         private Mat mask = new Mat();
//         private double lastDistance = -1; // To store the last detected distance
//         private Rect largestBoundingBox = null; // To store the largest detected bounding box
//         private boolean boundingBoxDetected = false; // Flag to check if a bounding box was detected
//         private int frameCount = 0; // Count frames to limit processing

//         // Reference to the AutoMode instance for telemetry
//         private AutoMode autoMode;

//         // Constructor that accepts the AutoMode instance
//         public ColorDetectionPipeline(AutoMode autoMode) {
//             this.autoMode = autoMode;
//         }

//         @Override
//         public Mat processFrame(Mat input) {
//             // Increment frame count
//             frameCount++;

//             // Only process every 5th frame
//             if (frameCount % FRAME_LIMIT != 0) {
//                 return input; // Return input without processing
//             }

//             // Convert the image to HSV
//             Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

//             // Create a mask for yellow color
//             Core.inRange(hsvImage, YELLOW_MIN, YELLOW_MAX, mask);

//             // Find contours in the mask
//             List<MatOfPoint> contours = new ArrayList<>();
//             Mat hierarchy = new Mat();
//             Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//             // Reset largest bounding box for each frame
//             largestBoundingBox = null;
//             double largestArea = 0;

//             // Process the contours to find bounding boxes
//             for (MatOfPoint contour : contours) {
//                 // Calculate bounding box
//                 Rect boundingBox = Imgproc.boundingRect(contour);
//                 double area = boundingBox.width * boundingBox.height; // Calculate area

//                 // Check if the area is larger than the current largest
//                 if (area > largestArea) {
//                     largestArea = area;
//                     largestBoundingBox = boundingBox;
//                 }
//             }

//             // If a largest bounding box was found, calculate the distance and update dimensions
//             if (largestBoundingBox != null) {
//                 // Draw the bounding box around the largest detected piece
//                 Imgproc.rectangle(input, largestBoundingBox.tl(), largestBoundingBox.br(), new Scalar(0, 255, 0), 2);

//                 // Calculate distance based on the pixel height of the bounding box
//                 double distance = calculateDistance(largestBoundingBox.height); // Change to height
//                 lastDistance = distance; // Update last detected distance

//                 // Set bounding box detected flag
//                 boundingBoxDetected = true;
//             } else {
//                 boundingBoxDetected = false; // No bounding box detected
//             }

//             // Return the original image with bounding boxes drawn
//             return input;
//         }

//         // Calculate distance to the object using the camera and the object's height
//         private double calculateDistance(double boundingBoxHeight) {
//             double blockSizeMM = 38.1; // Block size in mm
//             double focalLengthMM = 4.0; // Focal length in mm
//             double imageHeightPixels = 720; // Vertical resolution of the image
//             double sensorHeightMM = 2.02; // Approximate sensor height in mm for the Logitech C270

//             // Calculate distance in millimeters using the provided formula
//             double distanceMM = (2.375 * blockSizeMM * focalLengthMM * imageHeightPixels) / (boundingBoxHeight * sensorHeightMM);
//             telemetry.addLine(String.format("distance: %.2f", distanceMM));

//             return distanceMM / 25.4; // Convert from mm to inches
//         }

//         // Get the last detected distance in inches
//         public double getLastDistance() {
//             return lastDistance; // Already in inches
//         }

//         // Get the angle to the bounding box based on its position
//         public double getLastBoundingBoxAngle() {
//             if (largestBoundingBox != null) {
//                 double centerX = largestBoundingBox.x + (largestBoundingBox.width / 2.0);
//                 double frameCenterX = 1280 / 2.0; // Assuming 1280 width for the camera frame
//                 return (centerX - frameCenterX) / frameCenterX; // Normalize angle based on frame center
//             }
//             return 0; // No angle if no bounding box
//         }

//         // Check if a bounding box has been detected
//         public boolean hasDetectedBoundingBox() {
//             return boundingBoxDetected;
//         }
//     }
// }
