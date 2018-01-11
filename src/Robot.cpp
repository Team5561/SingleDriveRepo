/**
 * Example demonstrating the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the SetSensorDirection() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target velocity.
 *
 * Tweak the PID gains accordingly.
 */
#include "WPILib.h"
#include "CANTalon.h" /* necessary as of FRC2017, comment out for earlier seasons */
#include "DriverStation.h"

class Robot: public IterativeRobot {

  frc::SendableChooser<std::string> V_AutonOption;
  const std::string C_AutonOpt0 = "Off";
  const std::string C_AutonOpt1 = "5 RPM";
  const std::string C_AutonOpt2 = "20 RPM";
  const std::string C_AutonOpt3 = "60 RPM";
  const std::string C_AutonOpt4 = "100 RPM";
  std::string V_AutonSelected;

private:
  CANTalon * _talon = new CANTalon(0);
  LiveWindow *V_Lw;
  std::string _sb;
  int _loops = 0;
  double targetSpeed = 0.0;

  void RobotInit() {
    V_AutonOption.AddDefault(C_AutonOpt0, C_AutonOpt0);
    V_AutonOption.AddObject(C_AutonOpt1, C_AutonOpt1);
    V_AutonOption.AddObject(C_AutonOpt2, C_AutonOpt2);
    V_AutonOption.AddObject(C_AutonOpt3, C_AutonOpt3);
    V_AutonOption.AddObject(C_AutonOpt4, C_AutonOpt4);
    frc::SmartDashboard::PutData("Auto Modes", &V_AutonOption);

    /* first choose the sensor */
    _talon->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
    _talon->SetSensorDirection(true);
    _talon->ConfigEncoderCodesPerRev(1), // if using FeedbackDevice.QuadEncoder
    //_talon->ConfigPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

    /* set the peak and nominal outputs, 12V means full */
    _talon->ConfigNominalOutputVoltage(+0.0f, -0.0f);
    _talon->ConfigPeakOutputVoltage(+12.0f, -12.0f);
    /* set closed loop gains in slot0 */
    _talon->SelectProfileSlot(0);
    _talon->SetF(0.1097);
    _talon->SetP(0.22);
    _talon->SetI(0.02);
    _talon->SetD(0.02);
  }
  /**
   * This function is called periodically during operator control
   */
  void TeleopPeriodic() {
    V_AutonSelected = V_AutonOption.GetSelected();

    if (V_AutonSelected == C_AutonOpt1)
      {
      targetSpeed = 5;
      }
    else if (V_AutonSelected == C_AutonOpt2)
      {
      targetSpeed = 20;
      }
    else if (V_AutonSelected == C_AutonOpt3)
      {
      targetSpeed = 60;
      }
    else if (V_AutonSelected == C_AutonOpt4)
      {
      targetSpeed = 100;
      }
    else
      {
      targetSpeed = 0;
      }

    /* get gamepad axis */
    double motorOutput = _talon->GetOutputVoltage() / _talon->GetBusVoltage();
    int pulseWidthPos = _talon->GetPulseWidthPosition();
    int pulseWidthUs = _talon->GetPulseWidthRiseToRiseUs();
    int periodUs = _talon->GetPulseWidthRiseToRiseUs();
    int pulseWidthVel = _talon->GetPulseWidthVelocity();

    CANTalon::FeedbackDeviceStatus sensorStatus = _talon->IsSensorPresent(CANTalon::CtreMagEncoder_Absolute);
    bool sensorPluggedIn = (CANTalon::FeedbackStatusPresent == sensorStatus);

    /* prepare line to print */
    _sb.append("\tout:");
    _sb.append(std::to_string(motorOutput));
    _sb.append("\tspd:");
    _sb.append(std::to_string(_talon->GetSpeed()));
    /* while button1 is held down, closed-loop on target velocity */
    if (1) {
          /* Speed mode */
//      double targetSpeed = 85.0; /* Target Speed */

      _talon->SetControlMode(CANSpeedController::kSpeed);
          _talon->Set(targetSpeed); /* 1500 RPM in either direction */

      /* append more signals to print when in speed mode. */
      _sb.append("\terrNative:");
      _sb.append(std::to_string(_talon->GetClosedLoopError()));
      _sb.append("\ttrg:");
      _sb.append(std::to_string(targetSpeed));
      _sb.append("\tPulseWidthPos:");
      _sb.append(std::to_string(pulseWidthPos));
      _sb.append("\tPeriodUs:");
      _sb.append(std::to_string(periodUs));
      _sb.append("\tSensorConnected:");
      _sb.append(std::to_string(sensorPluggedIn));
        } else {
      /* Percent voltage mode */
      _talon->SetControlMode(CANSpeedController::kPercentVbus);
      _talon->Set(0.1);
    }
    /* print every ten loops, printing too much too fast is generally bad for performance */
    if (++_loops >= 10) {
      _loops = 0;
      printf("%s\n",_sb.c_str());
    }
    _sb.clear();

    SmartDashboard::PutNumber("Speed", _talon->GetSpeed());
    SmartDashboard::PutNumber("Output Voltage", _talon->GetOutputVoltage());
    SmartDashboard::PutNumber("Bus Voltage", _talon->GetBusVoltage());
  }
};

START_ROBOT_CLASS(Robot)
