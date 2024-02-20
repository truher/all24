package org.team100.frc2024;

import org.team100.frc2024.motion.intake.Intake;

public class RobotState100 {

    public enum State100{
        MANUAL, SHOOTING, AMPING, NONE
    }

    public enum ShooterState100{
        FEED, NONE, DEFAULTSHOOT, READYTOSHOOT, DOWN
    }

    public enum AmpState100{
        UP, DOWN, FEED, NONE
    }

    public enum IntakeState100{
        INTAKE, OUTTAKE, STOP, NONE
    }

    public static State100 currentRobotState = State100.SHOOTING;
    public static ShooterState100 currentShooterState = ShooterState100.DEFAULTSHOOT;
    public static AmpState100 currentAmpState = AmpState100.NONE;
    public static IntakeState100 currentIntakeState = IntakeState100.INTAKE;


    public void changeRobotState(State100 state){
        currentRobotState = state;
    }

    public static State100 getRobotState(){
        return currentRobotState;
    }

    public static void changeShooterState(ShooterState100 state){
        currentShooterState = state;
    }

    public static ShooterState100 getShooterState(){
        return currentShooterState;
    }

    public static void changeAmpState(AmpState100 state){
        currentAmpState = state;
    }

    public static AmpState100 getAmpState(){
        return currentAmpState;
    }

    public static void changeIntakeState(IntakeState100 state){
        currentIntakeState = state;
    }

    public static IntakeState100 getIntakeState(){
        return currentIntakeState;
    }
    
    
}
