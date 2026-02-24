package org.firstinspires.ftc.teamcode.main;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {
    private DcMotorEx motor;
    private final float dir;
    private double vel;
    public PID(HardwareMap hardwareMap, String motor_name, float motor_dir, double p, double i, double d, double f) {
        motor = hardwareMap.get(DcMotorEx.class, motor_name);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        dir = motor_dir / abs(motor_dir);
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public void setRPM(double rpm) {
        vel = rpm * 6 * dir;
        motor.setVelocity(vel, AngleUnit.DEGREES);
    };
    public double getRPM() {
        return motor.getVelocity(AngleUnit.DEGREES) / 6 * dir;
    }
}
