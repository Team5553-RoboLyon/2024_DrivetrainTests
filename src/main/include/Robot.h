/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Constants.h>
#include <frc/TimedRobot.h>
#if XBOX_CONTROLLER
#include <frc/XboxController.h>
#else
#include <frc/Joystick.h>
#endif
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <networktables/NetworkTableEntry.h>

#include "lib/CSVLogFile.h"
#include "lib/CustomMaths.h"
#include "lib/NL/NLPid.h"
#include "lib/NL/Characterization/NLMotorCharacterization.h"
#include "lib/NL/Characterization/NLCharacterization_Tests.h"
#include "Joystick.h"
#if IMU
#include <adi/ADIS16470_IMU.h>
#endif
#include <frc/PowerDistribution.h>
#include "lib/NL/NLTrajectoryStateSPack.h"

#include "lib/newCSVLogFile.h"
#include <units/time.h>

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 7   // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define JMAX 45
#define WMAX ((2.0 * VMAX) / TRACKWIDTH) // vitesse angulaire Max theorique
#define PATHNAME "0_8Great2UJ30A1_5V1"

// TEST *********************************************
#define TEST_LOWVOLTAGE_NB 10    // Nombre de tests ( subdivisions ) sur l'intervalle ]0,TEST_LOWVOLTAGE_MAX] volts						... 10 ou 20 ?
#define TEST_LOWVOLTAGE_MAX 0.15 // Volts

#define TEST_MEDIUMVOLTAGE_NB 5    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_LOWVOLTAGE_MAX,TEST_MEDIUMVOLTAGE_MAX] volts	... 20 ou 25 ?
#define TEST_MEDIUMVOLTAGE_MAX 1.0 // Volts

#define TEST_HIGHVOLTAGE_NB 44    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_MEDIUMVOLTAGE_MAX,TEST_HIGHVOLTAGE_MAX] volts... 12 ou 24 ?
#define TEST_HIGHVOLTAGE_MAX 12.0 // Volts

#define TIME_RAMP_CHARACTERIZATION 0.6
#define TIME_RAMP_VOLTAGE_CHARACTERIZATION 8

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void Drive(double forward, double turn);

private:
  NLPID m_pid;
  NLPID_ERROR m_errorLeft;
  NLPID_ERROR m_errorRight;

  Nf32 m_leftErrorVoltage;
  Nf32 m_rightErrorVoltage;
  Nf32 m_refLeftS;
  Nf32 m_refRightS;
  Nf32 m_prevS;
  Nf32 m_prevK;
  Nf32 m_estimatedAngle;
  Nf32 m_dsLeftWheel;
  Nf32 m_dsRightWheel;
  NLTRAJECTORY_STATE_S_PACK m_trajectoryStatesPack;
  NLTRAJECTORY_STATE_S m_currrentSState;
  NLMOTOR_CHARACTERIZATION m_motorCharacterization[4]; // droite: 0,1 gauche: 2,3

  double m_targetLeftSpeed;
  double m_targetRightSpeed;
  VA m_va_left;
  VA m_va_right;
  VA m_va_max;
  KineticToVoltage m_kv;

  CSVLogFile *m_LogFile, *m_LogFileDriving;
  nt::GenericEntry *m_LogFileName, *m_PowerEntry, *m_logGyro, *m_LogFilenameDriving, *m_speedY, *m_speedX, *m_customEntry;

  ctre::phoenix::motorcontrol::can::TalonFX m_moteurDroite{1};
  ctre::phoenix::motorcontrol::can::TalonFX m_moteurDroiteFollower{2};
  ctre::phoenix::motorcontrol::can::TalonFX m_moteurDroiteFollower2{3};
  ctre::phoenix::motorcontrol::can::TalonFX m_moteurGauche{4};
  ctre::phoenix::motorcontrol::can::TalonFX m_moteurGaucheFollower{5};
  ctre::phoenix::motorcontrol::can::TalonFX m_moteurGaucheFollower2{6};

  frc::PowerDistribution m_pdp;

  frc::Encoder m_encodeurExterneDroite{2, 3, false, frc::Encoder::k4X};
  frc::Encoder m_encodeurExterneGauche{0, 1, true, frc::Encoder::k4X};

#if IMU
  frc::ADIS16470_IMU m_imu{};
  frc::LinearFilter<double> filterX = frc::LinearFilter<double>::MovingAverage(64);
  frc::LinearFilter<double> filterY = frc::LinearFilter<double>::MovingAverage(64);
#endif

  char m_invertedPrefix[8];

  bool modeClimberJF = false;
  bool doigtLeve;
  bool shooterOn = false;
  bool m_override = false;
  bool m_isLogging = false;
  bool m_isPathFollowing = false;
  double m_ramp = 0;
  double m_time0;

  int m_logState = 0;
  char m_prefix[512];

  double init_x;
  double init_y;

  /*m_IsEnabledEntry = frc::Shuffleboard::GetTab("Shooter").Add("Is Shooter enabled", false).WithWidget(frc::BuiltInWidgets::kBooleanBox).GetEntry();
    m_PowerEntry = frc::Shuffleboard::GetTab("Shooter").Add("Power", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).GetEntry();
    m_LogEntry = frc::Shuffleboard::GetTab("Shooter").Add("Logging", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("Shooter").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
  */

  void LogData();
  double m_turnAdjustFactor = 1;

#if XBOX_CONTROLLER
  frc::XboxController m_driverController{0};
#else
  frc::Joystick m_leftHandController{0};
  frc::Joystick m_rightHandController{1};
#endif

  NLCharacterization_Tests m_motorCharacterizationTests{
      &m_moteurGauche,
      &m_moteurGaucheFollower,
      &m_moteurGaucheFollower2,
      &m_moteurDroite,
      &m_moteurDroiteFollower,
      &m_moteurDroiteFollower2,
      &m_encodeurExterneGauche,
      &m_encodeurExterneDroite,
      TEST_LOWVOLTAGE_NB,
      TEST_LOWVOLTAGE_MAX,
      TEST_MEDIUMVOLTAGE_NB,
      TEST_MEDIUMVOLTAGE_MAX,
      TEST_HIGHVOLTAGE_NB,
      TEST_HIGHVOLTAGE_MAX,
      TIME_RAMP_CHARACTERIZATION,
      TIME_RAMP_VOLTAGE_CHARACTERIZATION};
};