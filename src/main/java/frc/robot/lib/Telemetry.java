// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public final class Telemetry {
  private static boolean m_isAllTelemetryEnabled = false;

  /** Initializes default telemetry settings and topics with custom periodic updates */
  public static void start() {
    SmartDashboard.putBoolean("EnableAllTelemetry", m_isAllTelemetryEnabled);

    Robot.addCustomPeriodic(Telemetry::updateTelemetrySetting, 1);
    Robot.addCustomPeriodic(Telemetry::updateMatchTime, 0.2);
    Robot.addCustomPeriodic(Telemetry::updateFPGATimestamp, 3);
  }

  /** This periodic function supports enabling/disabling the full stream of telemetry pushed to LiveWindow. */
  private static void updateTelemetrySetting() {
    boolean isAllTelemetryEnabled = SmartDashboard.getBoolean("EnableAllTelemetry", m_isAllTelemetryEnabled);
    if (m_isAllTelemetryEnabled != isAllTelemetryEnabled) {
      m_isAllTelemetryEnabled = isAllTelemetryEnabled;
      if (m_isAllTelemetryEnabled) {
        LiveWindow.enableAllTelemetry();
      } else {
        LiveWindow.disableAllTelemetry();
      }
    }
  }

  /** This periodic function supports providing the current match time to the driver station dashboard app during competition mode. */
  private static void updateMatchTime() {
    SmartDashboard.putNumber("Timing/MatchTime", Math.floor(Timer.getMatchTime()));
  }

  /** This periodic function supports adding FPGA timestamps to entries when using the NetworkTables v3 protocol. */
  private static void updateFPGATimestamp() {
    SmartDashboard.putNumber("Timing/FPGATimestamp", Timer.getFPGATimestamp());
  }
}
