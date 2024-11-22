package frc.robot.subsystems.intake;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();

        return instance;
    }


    private CANSparkMax intakeBottom = new CANSparkMax(14, MotorType.kBrushless); 
    private CANSparkMax intakeTop = new CANSparkMax(13, MotorType.kBrushless);

    private RelativeEncoder topEnc = intakeTop.getEncoder(); 
    private RelativeEncoder botEnc = intakeBottom.getEncoder(); 

    public Intake() {
        intakeBottom.setSmartCurrentLimit(40); 
        intakeTop.setSmartCurrentLimit(40); 

        topEnc.setVelocityConversionFactor(1); 
        botEnc.setVelocityConversionFactor(1); 

        intakeTop.setOpenLoopRampRate(0.1);
        intakeBottom.setOpenLoopRampRate(0.1);

        intakeTop.enableVoltageCompensation(12); 
        intakeBottom.enableVoltageCompensation(12); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("top vel", topEnc.getVelocity());
        SmartDashboard.putNumber("bot vel", botEnc.getVelocity()); 
        
    }

    public void setBottom(double speed) {
        intakeBottom.set(speed);
    }

    public void setTop(double speed) {
        intakeTop.set(speed);
    }

    public Command topSpeed(double speed) {
        return new InstantCommand(() -> setTop(speed)); 
    }

    public Command bottomSpeed(double speed) {
        return new InstantCommand(() -> setBottom(speed)); 
    }

    public Command intakeNote(double speed) {
        return Commands.sequence(new InstantCommand(() -> setTop(-speed)), new InstantCommand(() -> setBottom(-speed))); 
    }

    public Command stopAll() {
        return intakeNote(0); 
    }

    public Command outtakeSpeaker(double topSpeed, double bottomSpeed) {
        return Commands.runOnce(() -> setTop(topSpeed), this)
        .andThen(Commands.waitSeconds(1))
        .andThen(Commands.runOnce((() -> setBottom(bottomSpeed)), this))
        .andThen(Commands.waitSeconds(1))
        .finallyDo(() -> {
            setTop(0);
            setBottom(0);
        }); 
    }
}