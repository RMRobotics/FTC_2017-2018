package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/**
 * Created by Kameron on 10/24/2017.
 */

//red easy
@Autonomous(name="jewelOnlyBlue", group ="woRMholeConfig")
@Disabled
public class autoMark4 extends LinearOpMode {


    private ColorSensor colorSensor;
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    //    private DcMotor arm;
    private Servo clawBL;
    private Servo clawBR;
    private Servo clawTL;
    private Servo clawTR;
    //    private Servo armT;
    private Servo gemBar;
//    private CRServo armB;

    private ElapsedTime time = new ElapsedTime();
    public static final String TAG = "Auto Version 1";
    /*    OpenGLMatrix lastLocation = null;*/
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        //declarations
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        lift = hardwareMap.dcMotor.get("lift");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        clawTL = hardwareMap.servo.get("clawTL");
        clawTR = hardwareMap.servo.get("clawTR");
        gemBar = hardwareMap.servo.get("gemBar");
        double /*timeToStance, timeToColumn,*/ rotate90; //rotate90 is the amount of time that it takes to rotate 90 degrees

        //init values
        gemBar.setDirection(Servo.Direction.REVERSE);
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setPower(0);
        clawBL.setPosition(-0.7);
        clawBR.setPosition(1);
        rotate90 = 0.4;//0.82
        colorSensor.enableLed(true);
//        arm = hardwareMap.dcMotor.get("arm");
//        armT = hardwareMap.servo.get("armT");
//        armB = hardwareMap.crservo.get("armB");
//        armB.setDirection(CRServo.Direction.FORWARD);
        clawTR.setPosition(1);
        clawTL.setPosition(-1);
//        armT.setPosition(0.5);
//        armB.setPower(0);
//        gemBar.setPosition(0);
//        timeToStance = 2;
//        timeToColumn = 0.75;



        //Activate Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AckoWtn/////AAAAGan7WAnq/0UVmQZG3sp7smBgRCNBnU1p+HmsTrC+W9TyxqaMlhFirDXglelvJCX4yBiO8oou6n7UWBfdRFbKHDqz0NIo5VcNHyhelmm0yK0vGKxoU0NZbQzjh5qVWnI/HRoFjM3JOq/LB/FTXgCcEaNGhXAqnz7nalixMeP8oRQlgX5nRVX4uE6w0K4yqIc5/FIDh1tn7PldiflmvNPhOW6FukPQD3d02wEnZB/JEchSSBzDbFA10XSgtYzXiweQI5tj+D5llLRrLh0mcWeouv55oSmya5RxUC26uEuO7bCAwyolWIuUr2Wh5oAG483nTD4vFhdjVMT7f0ovLO73C6xr2AXpNwen9IExRxBeosQ4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();
        waitForStart();

        while(opModeIsActive()) {
            //Grab the glyph
            //clawBL.setPosition(0.7);
            //clawBR.setPosition(0.3);
//            clawTR.setPosition(-0.2);
//            clawTL.setPosition(0.5);
//            holdUp(1.5);
//
//            //Lift the lift so the block doesn't drag
//            lift.setPower(-0.2);
//            holdUp(0.5);
//
//            //Stop lifting
//            lift.setPower(0);

            //Drop gemBar
            gemBar.setPosition(-0.92);
            holdUp(1.5);

            //Scan color of the jewel to the right
            boolean detected = false;
            String color = "";
            time.reset();
            while ((!detected) && (time.seconds()<5)) {
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.update();

                if ((colorSensor.blue() < 1) && (colorSensor.red()>0))
                {
                    detected = true;
                    color = "Red";
                }
                else if ((colorSensor.blue() > 0) && (colorSensor.red()<1))
                {
                    detected = true;
                    color = "Blue";
                }
                else
                    telemetry.addData("Not yet bois", "");

                telemetry.update();
            }
            holdUp(1);

            if (color.equals("")){
                gemBar.setPosition(1);
            }

            //Move backwards to topple jewel if necessary (if we need to move forward, it would be knocked off by going to read vuforia)
            if (color.equals("Red"))
            {
                //Knock off jewel
                move(0.18, -0.5, 0.0, 0.0);
                wheelBL.setPower(0);
                wheelBR.setPower(0);
                wheelFL.setPower(0);
                wheelFR.setPower(0);
                holdUp(1.5);

                //Retract gemBar and return to Start
                gemBar.setPosition(1);
                holdUp(3);
                move(0.22, 0.5, 0.0, 0.0);
                wheelBL.setPower(0);
                wheelBR.setPower(0);
                wheelFL.setPower(0);
                wheelFR.setPower(0);
                holdUp(2);
            }

            //Move to a spot where we can read Vuforia
            move(0.4, 0.5, 0.0, 0.0);
            wheelBL.setPower(0);
            wheelBR.setPower(0);
            wheelFL.setPower(0);
            wheelFR.setPower(0);
            holdUp(1);

            //If we went forward, retract gemBar
            if (color.equals("Blue")) {
                gemBar.setPosition(1);
                holdUp(3);
            }

            //Decode the Vuforia
//            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//            while (vuMark == RelicRecoveryVuMark.UNKNOWN){
//                vuMark = RelicRecoveryVuMark.from(relicTemplate);
//                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//                    telemetry.addData("VuMark", "%s visible", vuMark);
//                    telemetry.update();
//                }
//            }
//
//            //Get in front of respective Cryptobox
//            if (vuMark.equals(RelicRecoveryVuMark.LEFT))
//                move(0.65, 0.5, 0.0, 0.0);
//            if (vuMark.equals(RelicRecoveryVuMark.CENTER))
//                move(0.55, 0.5, 0.0, 0.0);
//            if (vuMark.equals(RelicRecoveryVuMark.RIGHT))
//                move(0.33, 0.5, 0.0, 0.0);
//
//            //Orient to face Cryptobox
//            move(rotate90, 0.05, 0.0, 90.0);
//
//            //Go in Cryptobox and release glyphs
//            move(0.6, 0.25, 0, 0);
//            clawTR.setPosition(1);
//            clawTL.setPosition(-1);
//            clawBL.setPosition(-0.7);
//            clawBR.setPosition(1);
//
//            //Move back
//            move(0.3, -0.5, 0, 0);
//            wheelBL.setPower(0);
//            wheelBR.setPower(0);
//            wheelFL.setPower(0);
//            wheelFR.setPower(0);

            //Fin
            break;
        }

    }

    //moves for duration seconds at power power, strafing at angle angle and rotating at angle rotate
    public void move(double duration, double power, double angle, double rotate) {
        angle *= (Math.PI / 180);
        time.reset();
        while (time.seconds() < duration) {
            wheelFL.setPower(power * Math.sin(angle + (Math.PI / 4)) + rotate);
            wheelFR.setPower(power * Math.cos(angle + (Math.PI / 4)) - rotate);
            wheelBL.setPower(power * Math.cos(angle + (Math.PI / 4)) + rotate);
            wheelBR.setPower(power * Math.sin(angle + (Math.PI / 4)) - rotate);
        }
    }

    //Pauses for time num
    public void holdUp(double num)
    {
        time.reset();
        while (time.seconds() < num) {}
    }
}