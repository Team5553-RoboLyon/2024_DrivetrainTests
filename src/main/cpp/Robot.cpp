/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Constants.h"
#include "Robot.h"
#include "lib/NL/NLOdometry.h"
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <time.h>
// #include <units/units.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/SmallString.h>

double Deadband(double value, double deadband = 0.1)
{
    if (std::abs(value) < deadband)
        return 0;
    else
        return value < 0 ? (value + deadband) / (1.0 - deadband) : (value - deadband) / (1.0 - deadband);
}

void Robot::Drive(double forward, double turn)
{
    // cout<<forward<<std::endl;
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.2);

    /*
    double c = 0.35 * (turn * 5.0 * (abs(turn) + 1) / (abs(forward) + 1));
    if (turn < 0.0) {
        m_drivetrain->Drive(forward * ((c + 1) / (1 - c)), forward);
    } else {
        m_drivetrain->Drive(forward, forward * ((1 - c) / (c + 1)));
    }*/

    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    // cout<<lwheel<<std::endl;

    m_moteurGauche.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, lwheel);
    m_moteurGaucheFollower.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, lwheel);
    m_moteurGaucheFollower2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, lwheel);
    m_moteurDroite.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rwheel);
    m_moteurDroiteFollower.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rwheel);
    m_moteurDroiteFollower2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rwheel);
}

void Robot::RobotInit()
{
    std::cout << "RobotInit Step 1" << std::endl;
    m_moteurDroite.ConfigFactoryDefault();
    m_moteurGauche.ConfigFactoryDefault();
    m_moteurGaucheFollower2.ConfigFactoryDefault();
    m_moteurDroiteFollower2.ConfigFactoryDefault();
    m_moteurDroiteFollower.ConfigFactoryDefault();
    m_moteurGaucheFollower.ConfigFactoryDefault();

    m_moteurDroite.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0)); // limite de courant
    m_moteurDroiteFollower.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));
    m_moteurDroiteFollower2.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));
    m_moteurGauche.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));
    m_moteurGaucheFollower.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));
    m_moteurGaucheFollower2.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));

#ifdef TIME_RAMP
    m_moteurDroite.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGauche.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower2.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower2.SetOpenLoopRampRate(TIME_RAMP);
#else
    m_moteurDroite.ConfigOpenloopRamp(0);
    m_moteurGauche.ConfigOpenloopRamp(0);
    m_moteurDroiteFollower.ConfigOpenloopRamp(0);
    m_moteurGaucheFollower.ConfigOpenloopRamp(0);
    m_moteurDroiteFollower2.ConfigOpenloopRamp(0);
    m_moteurGaucheFollower2.ConfigOpenloopRamp(0);
#endif
#if VOLTAGE_COMPENSATION
    m_moteurDroite.EnableVoltageCompensation(true);
    m_moteurGauche.EnableVoltageCompensation(true);
    m_moteurDroiteFollower.EnableVoltageCompensation(true);
    m_moteurGaucheFollower.EnableVoltageCompensation(true);
    m_moteurDroiteFollower2.EnableVoltageCompensation(true);
    m_moteurGaucheFollower2.EnableVoltageCompensation(true);

    m_moteurDroite.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGauche.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower2.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower2.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_VALUE);
#else
    m_moteurGauche.DisableVoltageCompensation();
    m_moteurGaucheFollower.DisableVoltageCompensation();
    m_moteurDroite.DisableVoltageCompensation();
    m_moteurDroiteFollower.DisableVoltageCompensation();
    m_moteurGaucheFollower2.DisableVoltageCompensation();
    m_moteurDroiteFollower2.DisableVoltageCompensation();
    µ
#endif

    m_moteurDroite.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_moteurGauche.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_moteurDroiteFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_moteurGaucheFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_moteurDroiteFollower2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_moteurGaucheFollower2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    std::cout << "RobotInit Step 2" << std::endl;
    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_LogFileName = frc::Shuffleboard::GetTab("voltage").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_customEntry = frc::Shuffleboard::GetTab("voltage").Add("Data", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#if IMU
    m_speedY = frc::Shuffleboard::GetTab("voltage").Add("speedY", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_speedX = frc::Shuffleboard::GetTab("voltage").Add("speedX", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#endif
    std::cout << "RobotInit Step 3" << std::endl;
    m_encodeurExterneDroite.SetReverseDirection(false);
    m_encodeurExterneGauche.SetReverseDirection(false);

    m_encodeurExterneDroite.SetDistancePerPulse(1);
    m_encodeurExterneGauche.SetDistancePerPulse(1);

    m_encodeurExterneDroite.SetSamplesToAverage(65);
    m_encodeurExterneGauche.SetSamplesToAverage(65);
    std::cout << "RobotInit Step 4" << std::endl;

    m_moteurDroite.SetInverted(true);
    m_moteurDroiteFollower.SetInverted(true);
    m_moteurDroiteFollower2.SetInverted(true);

    m_moteurGauche.SetInverted(false);
    m_moteurGaucheFollower.SetInverted(false);
    m_moteurGaucheFollower2.SetInverted(false);

    m_moteurDroite.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 4);
    m_moteurDroite.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 4);
    m_moteurDroite.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 50);

    m_moteurGauche.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 4);
    m_moteurGauche.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 4);
    m_moteurGauche.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 50);

    m_moteurDroiteFollower.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 4);
    m_moteurDroiteFollower.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 4);
    m_moteurDroiteFollower.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 50);

    m_moteurDroiteFollower2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 4);
    m_moteurDroiteFollower2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 4);
    m_moteurDroiteFollower2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 50);

    m_moteurGaucheFollower.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 4);
    m_moteurGaucheFollower.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 4);
    m_moteurGaucheFollower.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 50);

    m_moteurGaucheFollower2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 4);
    m_moteurGaucheFollower2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 4);
    m_moteurGaucheFollower2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 50);

    std::cout << "RobotInit Step 5" << std::endl;
    Robot::AddPeriodic([&]()
                       { m_motorCharacterizationTests.fastLoop(); },
                       2_ms, 1_ms);
    std::cout << "RobotInit Step 6" << std::endl;
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    m_encodeurExterneDroite.Reset();
    m_encodeurExterneGauche.Reset();

    m_isLogging = 0;
    m_isPathFollowing = 0;

    // reset robot path
    m_dsLeftWheel = 0.0f;
    m_dsRightWheel = 0.0f;

    m_currrentSState.null();

    m_refLeftS = 0.0f;
    m_refRightS = 0.0f;
    m_prevK = 0.0f;
    m_prevS = 0.0f;

    m_errorLeft.reset();
    m_errorRight.reset();

#if IMU
    init_x = m_imu.GetAccelInstantX();
    init_y = m_imu.GetAccelInstantY();
#endif
}

void Robot::TeleopPeriodic()
{
    std::cout << m_encodeurExterneGauche.Get() << std::endl;
    // int errorLeftState = m_moteurGauche.GetStickyFaults();
    // int errorLeftFollowerState = m_moteurGaucheFollower.GetStickyFaults();
    // int errorLeftFollowe2rState = m_moteurGaucheFollower2.GetStickyFaults();

    // int errorRightState = m_moteurDroite.GetStickyFaults();
    // int errorRightFollowerState = m_moteurDroiteFollower.GetStickyFaults();
    // int errorRightFollower2State = m_moteurDroiteFollower2.GetStickyFaults();

    // frc::Shuffleboard::GetTab("voltage").Add("errorLeftState", errorLeftState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorLeftFollowerState", errorLeftFollowerState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorLeftFollower2State", errorLeftFollowe2rState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorRightState", errorRightState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorRightFollowerState", errorRightFollowerState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorRightFollower2State", errorRightFollower2State).WithWidget(frc::BuiltInWidgets::kTextView);

    char infos[256];
    char desc[256];
#if IMU
    m_speedY.SetDouble(filterY.Calculate(m_imu.GetAccelInstantY() - init_y));
    m_speedX.SetDouble(filterX.Calculate(m_imu.GetAccelInstantX() - init_x));
#endif
    if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
    {
        Drive(-m_driverController.GetLeftY(), m_driverController.GetRightX());
    }

    if (m_driverController.GetBButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {
            m_motorCharacterizationTests.nextTest();
            sprintf(infos, "%s En Attente ... Appuyer sur A pour Démarrer.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
    }

    if (m_driverController.GetXButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {

            m_motorCharacterizationTests.previousTest();
            sprintf(infos, "%s En Attente ... Appuyer sur A pour Démarrer.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
    }

    if (m_driverController.GetAButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Started)
        {
            m_motorCharacterizationTests.stop();
            m_motorCharacterizationTests.nextTest();
            sprintf(infos, "%s En Attente ... Appuyer sur A pour Démarrer.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
        else if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {
            m_motorCharacterizationTests.start();
            sprintf(infos, "%s En Cours ... Appuyer sur A pour Arrêter.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
    }

    m_LogFileName->SetString(m_motorCharacterizationTests.getCurrentFileLogName(infos, 256));
}

void Robot::TestInit()
{
}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif