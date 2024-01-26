// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Encoder.h>
#include <assert.h>
#include "lib/CSVLogFile.h"
#include "lib/N/NMath.h"
#include "lib/N/NType.h"
#include "lib/N/NFlags.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>
// #include <units/units.h>
#include <iostream>

typedef struct TestSpecs TestSpecs;
struct TestSpecs
{
  unsigned long m_flags;
  double m_voltage;
  double m_ramp;
};

class NLCharacterization_Tests
{

public:
  enum class State
  {
    Stopped = 0,
    AskForStart = 1,
    Started = 2,
    AskForStop = 3
  };
  NLCharacterization_Tests(ctre::phoenix::motorcontrol::can::TalonFX *leftMotor,
                           ctre::phoenix::motorcontrol::can::TalonFX *leftMotorFollower,
                           ctre::phoenix::motorcontrol::can::TalonFX *leftMotorFollower2,
                           ctre::phoenix::motorcontrol::can::TalonFX *rightMotor,
                           ctre::phoenix::motorcontrol::can::TalonFX *rightMotorFollower,
                           ctre::phoenix::motorcontrol::can::TalonFX *rightMotorFollower2,
                           frc::Encoder *externalEncoderLeft,
                           frc::Encoder *externalEncoderRight,
                           long nbTestLow,
                           double endVoltageLow,
                           long nbTestMedium,
                           double endVoltageMedium,
                           long nbTestHigh,
                           double endVoltageHigh,
                           double rampValue,
                           double rampVoltage);
  ~NLCharacterization_Tests();
  void nextTest();
  void previousTest();
  void setCurrentTest(uint8_t testId);
  void start();
  void stop();
  void fastLoop();
  State getState();
  uint8_t getCurrentTestId();
  char *getCurrentFileLogName(char *pbuffer, uint size);
  char *getCurrentTestDescription(char *pmessage, uint size_terminated_null_char_included);
  uint getTestsCounter();
  uint areAllTestsDone();

private:
  ctre::phoenix::motorcontrol::can::TalonFX *m_rightMotor;
  ctre::phoenix::motorcontrol::can::TalonFX *m_rightMotorFollower;
  ctre::phoenix::motorcontrol::can::TalonFX *m_rightMotorFollower2;
  ctre::phoenix::motorcontrol::can::TalonFX *m_leftMotor;
  ctre::phoenix::motorcontrol::can::TalonFX *m_leftMotorFollower;
  ctre::phoenix::motorcontrol::can::TalonFX *m_leftMotorFollower2;

  frc::Encoder *m_externalEncoderRight;
  frc::Encoder *m_externalEncoderLeft;

  TestSpecs *TestData;
  State m_state = State::Stopped;
  uint8_t m_CurrentTestID = 0;
  double m_oldRamp;
  uint8_t m_nbTotalTest;

  CSVLogFile *m_LogFile;

  double m_ramp = 0;
  double m_time0;
};
