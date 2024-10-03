package frc.robot.auto.lyAuto.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Telemetry {
    private static Telemetry instance;
    private NetworkTableInstance insNT;
    private NetworkTable SmartDtable;
    private NetworkTable TitanDtable;

    public static final class Constants{
        public static final String SMART_DASHBOARD = "SmartDashboard";
        public static final String TITAN_DASHBOARD = "TitanDash";
    }

    public static Telemetry getInstance() {
        if (instance == null) instance = new Telemetry();
        return instance;
    }

    private Telemetry() {
        this.insNT = NetworkTableInstance.getDefault();
        this.SmartDtable = insNT.getTable(Constants.SMART_DASHBOARD); //TODO: CHECK IF RIGHT FOR SMART DASHBOARD
        this.TitanDtable = insNT.getTable(Constants.TITAN_DASHBOARD);
    }

    
}
