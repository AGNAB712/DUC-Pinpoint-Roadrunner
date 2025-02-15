package org.firstinspires.ftc.teamcode.DUC_auto;

import static org.firstinspires.ftc.teamcode.lib.Hardware.closeClawAngle;
import static org.firstinspires.ftc.teamcode.lib.Hardware.openClawAngle;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "TEST PRECISION", group = "Autonomous")
public class TEST_AUTO_4 extends LinearOpMode {

    int armTickPosition = 0;
    int normalArmTickPosition = 0;
    int spoolTickPosition = 0;

    public static int armAddition = 2;
    public static double ypos1 = 0.5;
    public static double xpos1 = 0.5;
    public static double ypos2 = 2;
    public static double xpos2 = 1.5;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(32, 64, Math.toRadians(270));
        MecanumDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Spool spool = new Spool(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        NormalClaw normalClaw = new NormalClaw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        NormalArm normalArm = new NormalArm(hardwareMap);
        TimerActions time = new TimerActions();
        drive.precision = 0;
        drive.precisionTime = 0;
        class PrecisionActions {
            class Precise implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    drive.precision = 0.5;
                    drive.precisionTime = 1;
                    return false;
                }
            }
            class VeryPrecise implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    drive.precision = 0.1;
                    drive.precisionTime = 3;
                    return false;
                }
            }
            class NotPrecise implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    drive.precision = 0;
                    drive.precisionTime = 0;
                    return false;
                }
            }
            public Action precise() {
                return new Precise();
            }
            public Action veryPrecise() {
                return new VeryPrecise();
            }
            public Action notPrecise() {
                return new NotPrecise();
            }
        }
        PrecisionActions precisionController = new PrecisionActions();

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(initialPose) //from initial pose to high basket
                .strafeTo(new Vector2d(32, 25))
                .strafeTo(new Vector2d(32, 64));;



        Actions.runBlocking(claw.openClaw());
        Actions.runBlocking(normalClaw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                trajectory1.build(),
                                new SleepAction(1),
                                precisionController.precise(),
                                trajectory1.build()
                            ),
                        arm.keepPosition(),
                        normalArm.keepPosition(),
                        spool.keepPosition()
                )

        );

    }

    public static class TimerActions {
        public static ElapsedTime timer = new ElapsedTime();
        public TimerActions() {
            timer.reset();
        }
        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                return false;
            }
        }
        public Action resetTimer() {
            return new ResetTimer();
        }

        public static double getTime() {
            return timer.milliseconds();
        }
    }


    public class Spool {
        private Motor spool;

        public Spool(HardwareMap hardwareMap) {
            spool = new Motor(hardwareMap, "spool");
            spool.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            spool.resetEncoder();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 2000;
                return false;
            }
        }
        public Action highBasket() {
            return new HighBasket();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 0;
                return false;
            }
        }
        public Action low() {
            return new Low();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spool.setRunMode(Motor.RunMode.PositionControl);
                spool.setPositionCoefficient(0.01);
                spool.setTargetPosition(spoolTickPosition);
                spool.set(1);
                spool.setPositionTolerance(20);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

    }

    public class Arm {
        private Motor arm;

        public Arm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "specimenArm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(armTickPosition);
                arm.set(0.75);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighRung implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 2000;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highRung() {
            return new HighRung();
        }
        public class HighRung2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1300;
                packet.put("arm position", arm.getCurrentPosition());
                packet.put("arm target", armTickPosition);
                packet.put("arm at position", arm.atTargetPosition());
                return arm.atTargetPosition() || time >= 3000;
            }
        }
        public Action highRung2() {
            return new HighRung2();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 0;
                double time = TimerActions.getTime();
                packet.put("timer", time);
                packet.put("armPosition", arm.getCurrentPosition());
                packet.put("arm target position", armTickPosition);
                packet.put("arm at position", arm.atTargetPosition());
                return arm.atTargetPosition() || time >= 3000;
            }
        }
        public Action low() {
            return new Low();
        }
    }
    public class Claw {
        private ServoEx claw;

        public Claw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "specimenClaw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class NormalClaw {
        private ServoEx claw;

        public NormalClaw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "claw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                if (claw.getAngle() == closeClawAngle) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                if (claw.getAngle() == openClawAngle) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }


    public class NormalArm {
        private Motor arm;

        public NormalArm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "arm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.setInverted(true);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(normalArmTickPosition);
                arm.set(0.75);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 2500;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highBasket() {
            return new HighBasket();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 555 + armAddition;
                keepPosition();
                if (arm.atTargetPosition()) {
                    keepPosition();
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action mid() {
            return new Mid();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 20;
                time = TimerActions.getTime();
                if (arm.atTargetPosition() || time >= 500) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new Low();
        }
    }

}

