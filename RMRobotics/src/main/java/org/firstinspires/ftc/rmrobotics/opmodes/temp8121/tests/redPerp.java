package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="redPerp", group ="woRMholeConfig")
public class redPerp extends LinearOpMode {


    private ColorSensor colorSensor;
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    private DcMotor arm;
    private Servo clawBL;
    private Servo clawBR;
    private Servo clawTL;
    private Servo clawTR;
    private Servo armT;
    private Servo gemBar1;
    private Servo gemBar2;
    private CRServo armB;

    private ElapsedTime time = new ElapsedTime();
    public static final String TAG = "Auto Version 1";
    /*    OpenGLMatrix lastLocation = null;*/
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);


        double /*timeToStance, timeToColumn,*/ rotate90; //rotate90 is the amount of time that it takes to rotate 90 degrees
        rotate90 = 0.4;//0.82

        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setPower(0);

        arm = hardwareMap.dcMotor.get("arm");
        armT = hardwareMap.servo.get("armT");
        armB = hardwareMap.crservo.get("armB");
        armB.setDirection(CRServo.Direction.FORWARD);
        armT.setPosition(0.5);
        armB.setPower(0);

        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        clawTL = hardwareMap.servo.get("clawTL");
        clawTR = hardwareMap.servo.get("clawTR");
        clawBL.setPosition(0.1);
        clawBR.setPosition(0.5);
        clawTR.setPosition(0.7);
        clawTL.setPosition(0.1);

        gemBar1 = hardwareMap.servo.get("gemBar1");
        gemBar2 = hardwareMap.servo.get("gemBar2");
        gemBar1.setPosition(0);
        gemBar2.setPosition(0);




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
            clawTR.setPosition(-0.1);
            clawTL.setPosition(0.9);
            holdUp(1.5);

            //Lift the lift so the block doesn't drag
            lift.setPower(0.2);
            holdUp(0.5);

            //Stop lifting
            lift.setPower(0);

            //Drop gemBar
            gemBar1.setPosition(1);
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

                if ((colorSensor.blue() > 0) && (colorSensor.red()<1))
                {
                    detected = true;
                    color = "Blue";
                }
                else if ((colorSensor.blue() < 1) && (colorSensor.red()>0))
                {
                    detected = true;
                    color = "Red";
                }
                else
                    telemetry.addData("Not yet bois", "");

                telemetry.update();
            }
            holdUp(1);

            if (color.equals("")){
                gemBar1.setPosition(0);
            }

            //Move backwards to topple jewel if necessary (if we need to move forward, it would be knocked off by going to read vuforia)
            else if (color.equals("Red"))
            {
                //Knock off jewel
                gemBar2.setPosition(1);
                holdUp(1.5);

                //Retract gemBars
                gemBar1.setPosition(0);
                holdUp(1.5);
                gemBar2.setPosition(0);
            }

            else {
                //Knock off jewel
                gemBar2.setPosition(-1);
                holdUp(1.5);

                //Retract gemBars
                gemBar1.setPosition(0);
                holdUp(1.5);
                gemBar2.setPosition(0);
            }

            holdUp(1.5);
            //Move to a spot where we can read Vuforia
            move(0.4, 0.5, 0.0, 0.0);
            wheelBL.setPower(0);
            wheelBR.setPower(0);
            wheelFL.setPower(0);
            wheelFR.setPower(0);
            holdUp(1);

            //Decode the Vuforia
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while (vuMark == RelicRecoveryVuMark.UNKNOWN){
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    telemetry.update();
                    parameters = new VuforiaLocalizer.Parameters();
                }
            }

            //move off stone
            move(0.3, 0.5, 0.0, 0.0);

            //stafe to correct collumn
            if (vuMark.equals(RelicRecoveryVuMark.LEFT))
                move(0.70, 0.5, 270.0, 0.0);
            if (vuMark.equals(RelicRecoveryVuMark.CENTER))
                move(0.55, 0.5, 270.0, 0.0);
            if (vuMark.equals(RelicRecoveryVuMark.RIGHT))
                move(0.33, 0.5, 270.0, 0.0);

            //Go in Cryptobox and release glyphs
            move(0.3, 0.25, 0, 0);
            clawTR.setPosition(0.7);
            clawTL.setPosition(0.1);
            clawBL.setPosition(0.1);
            clawBR.setPosition(0.5);

            //Move back
            move(0.3, -0.5, 0, 0);
            wheelBL.setPower(0);
            wheelBR.setPower(0);
            wheelFL.setPower(0);
            wheelFR.setPower(0);

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