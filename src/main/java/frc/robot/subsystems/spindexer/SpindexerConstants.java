package frc.robot.subsystems.spindexer;

public final class SpindexerConstants {
    public static final int OFFRPM = 0;
    public static final int RPMTOLERANCE = 50; // TODO placeholder
    public static final int RAMPRATE = 2; //seconds to full power

    // SpinDexer Velo: (Find Max Velocity at Motor 100% -> put that in RPM MAX)
    
    public static final int FEED_RPM = 1000; // TODO placeholder
    public static final int MAX_RPM = 6400;

    // Closed-loop velocity PID — TODO tune on robot
    public static final double SPIN_kP = 0.0001;
    public static final double SPIN_kI = 0.0;
    public static final double SPIN_kD = 0.0;
    public static final double SPIN_kS = 0.20;
    public static final double SPIN_kV = 12.0 / MAX_RPM;

    // SpinDexer Motor Configs
    public static final int SPINDEXER_ID = 17; // Vortex (Spark Flex)

    public static final int SPIN_STATOR_CURRENT_LIMIT = 60;
    public static final int SPIN_SUPPLY_CURRENT_LIMIT = 40;
    // true = coast, false = brake
    public static final boolean SPIN_COAST = true;
}
