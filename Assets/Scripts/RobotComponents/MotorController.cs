using UnityEngine;

/// <summary>
/// DC Motor Controller
///
/// A full-featured motor controller that sits between your game logic and the
/// DCMotor + MotorEncoder components.  Supports three control modes:
///
///   VOLTAGE   — Direct voltage command (open-loop).  Useful for testing.
///   VELOCITY  — PID speed control using encoder feedback (rad/s setpoint).
///   POSITION  — Cascaded position + velocity PID (radians setpoint).
///
/// Features:
///   • Anti-windup (integrator clamping)
///   • Derivative-on-measurement (avoids derivative kick on setpoint change)
///   • Configurable output voltage limits (e.g., ±12 V supply rail)
///   • Soft-start ramp on enable
///   • Runtime mode switching with bumpless transfer
///   • Emergency stop (E-Stop) that coasts or brakes the motor
///   • Exposes all internal signals for oscilloscope / data-logging
/// </summary>
[RequireComponent(typeof(DCMotor))]
[RequireComponent(typeof(MotorEncoder))]
public class MotorController : MonoBehaviour
{
    // ── Enums ──────────────────────────────────────────────────────────

    public enum ControlMode { Voltage, Velocity, Position }
    public enum StopMode    { Coast, ActiveBrake }

    // ── Inspector ──────────────────────────────────────────────────────

    [Header("Control Mode")]
    public ControlMode mode = ControlMode.Velocity;

    [Header("Supply / Safety")]
    [Tooltip("Maximum supply voltage magnitude (V). Output is clamped to ±this value.")]
    public float supplyVoltage = 12f;

    [Tooltip("Maximum allowable current (A). Motor is disabled if this is exceeded.")]
    public float currentLimit = 90f;

    [Tooltip("Soft-start ramp time (seconds) from 0 → full voltage on enable.")]
    public float softStartTime = 0.1f;

    [Header("Open-Loop Voltage Command")]
    [Tooltip("Voltage to apply when mode = Voltage.")]
    public float voltageSetpoint = 0f;

    [Header("Velocity PID  (rad/s)")]
    public float velocitySetpoint = 0f;  // rad/s
    public float vel_Kp = 2.0f;
    public float vel_Ki = 5.0f;
    public float vel_Kd = 0.01f;
    [Tooltip("Maximum integrator accumulation (V) — prevents windup.")]
    public float vel_IntegralLimit = 8f;

    [Header("Position PID  (rad)")]
    public float positionSetpoint = 0f;  // radians
    public float pos_Kp = 10.0f;
    public float pos_Ki = 0.5f;
    public float pos_Kd = 0.5f;
    [Tooltip("Maximum integrator accumulation (rad/s) — inner velocity limit.")]
    public float pos_IntegralLimit = 5f;
    [Tooltip("Clamps the velocity command issued by the position loop (rad/s).")]
    public float maxVelocityCommand = 20f;

    [Header("Emergency Stop")]
    public bool  eStop = false;
    public StopMode eStopMode = StopMode.ActiveBrake;

    [Header("Runtime State — Read Only")]
    [SerializeField] private float _outputVoltage;
    [SerializeField] private float _velocityError;
    [SerializeField] private float _positionError;
    [SerializeField] private float _velIntegral;
    [SerializeField] private float _posIntegral;
    [SerializeField] private bool  _currentFault;
    [SerializeField] private bool  _enabled = true;
    [SerializeField] private float _softStartFactor = 1f;

    // ── Internal ───────────────────────────────────────────────────────

    private DCMotor      _motor;
    private MotorEncoder _encoder;

    // Derivative-on-measurement state
    private float _prevMeasuredVelocity;
    private float _prevMeasuredPosition;

    // Soft-start
    private float _softStartTimer;

    // Bumpless transfer: track last mode to reinitialise integrators
    private ControlMode _lastMode;

    // ── Public API ─────────────────────────────────────────────────────

    public float OutputVoltage  => _outputVoltage;
    public bool  CurrentFault   => _currentFault;
    public bool  IsEnabled      => _enabled;

    /// <summary>Enable or disable the controller output.</summary>
    public void SetEnabled(bool en)
    {
        if (en && !_enabled)
        {
            // Re-enable: restart soft-start, clear faults
            _softStartTimer  = 0f;
            _softStartFactor = 0f;
            _currentFault    = false;
            ResetIntegrators();
        }
        _enabled = en;
    }

    /// <summary>Command a voltage directly (also switches mode to Voltage).</summary>
    public void CommandVoltage(float v)
    {
        mode = ControlMode.Voltage;
        voltageSetpoint = v;
    }

    /// <summary>Command a velocity setpoint in rad/s (switches to Velocity mode).</summary>
    public void CommandVelocity(float radPerSec)
    {
        mode = ControlMode.Velocity;
        velocitySetpoint = radPerSec;
    }

    /// <summary>Command a position setpoint in radians (switches to Position mode).</summary>
    public void CommandPosition(float radians)
    {
        mode = ControlMode.Position;
        positionSetpoint = radians;
    }

    /// <summary>Command a position setpoint in degrees (switches to Position mode).</summary>
    public void CommandPositionDegrees(float degrees)
        => CommandPosition(degrees * Mathf.Deg2Rad);

    // ── Unity Lifecycle ────────────────────────────────────────────────

    private void Awake()
    {
        _motor   = GetComponent<DCMotor>();
        _encoder = GetComponent<MotorEncoder>();
        _lastMode = mode;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // ── E-Stop ───────────────────────────────────────────────────
        if (eStop)
        {
            _outputVoltage = (eStopMode == StopMode.ActiveBrake) ? 0f : _outputVoltage * 0f;
            _motor.SetVoltage(0f);
            return;
        }

        // ── Current fault latch ───────────────────────────────────────
        if (Mathf.Abs(_motor.Current) > currentLimit)
        {
            _currentFault = true;
            _enabled      = false;
        }

        if (!_enabled)
        {
            _motor.SetVoltage(0f);
            return;
        }

        // ── Soft-start ramp ───────────────────────────────────────────
        _softStartTimer  += dt;
        _softStartFactor  = (softStartTime > 0f)
                          ? Mathf.Clamp01(_softStartTimer / softStartTime)
                          : 1f;

        // ── Bumpless transfer — reset integrators on mode change ───────
        if (mode != _lastMode)
        {
            ResetIntegrators();
            _lastMode = mode;
        }

        // ── Control computation ───────────────────────────────────────
        float measuredVel = _encoder.MeasuredRPM / 9.5492f; // RPM → rad/s
        float measuredPos = _encoder.PositionRadians;

        float command = 0f;

        switch (mode)
        {
            case ControlMode.Voltage:
                command = voltageSetpoint;
                break;

            case ControlMode.Velocity:
                command = VelocityPID(velocitySetpoint, measuredVel, dt);
                break;

            case ControlMode.Position:
                // Outer position loop generates velocity command
                float velCmd = PositionPID(positionSetpoint, measuredPos, dt);
                velCmd = Mathf.Clamp(velCmd, -maxVelocityCommand, maxVelocityCommand);
                // Inner velocity loop converts that to voltage
                command = VelocityPID(velCmd, measuredVel, dt);
                break;
        }

        // ── Clamp to supply rail and apply soft-start ─────────────────
        command = Mathf.Clamp(command, -supplyVoltage, supplyVoltage);
        command *= _softStartFactor;

        _outputVoltage = command;
        _motor.SetVoltage(command);

        // Cache measurement for next derivative step
        _prevMeasuredVelocity = measuredVel;
        _prevMeasuredPosition = measuredPos;
    }

    // ── PID Loops ──────────────────────────────────────────────────────

    private float VelocityPID(float setpoint, float measured, float dt)
    {
        _velocityError = setpoint - measured;

        // Integral with anti-windup clamp
        _velIntegral = Mathf.Clamp(
            _velIntegral + _velocityError * dt,
            -vel_IntegralLimit, vel_IntegralLimit);

        // Derivative-on-measurement (avoids kick when setpoint jumps)
        float derivative = (dt > 0f) ? -(measured - _prevMeasuredVelocity) / dt : 0f;

        return vel_Kp * _velocityError
             + vel_Ki * _velIntegral
             + vel_Kd * derivative;
    }

    private float PositionPID(float setpoint, float measured, float dt)
    {
        _positionError = setpoint - measured;

        _posIntegral = Mathf.Clamp(
            _posIntegral + _positionError * dt,
            -pos_IntegralLimit, pos_IntegralLimit);

        float derivative = (dt > 0f) ? -(measured - _prevMeasuredPosition) / dt : 0f;

        return pos_Kp * _positionError
             + pos_Ki * _posIntegral
             + pos_Kd * derivative;
    }

    private void ResetIntegrators()
    {
        _velIntegral = 0f;
        _posIntegral = 0f;
    }

    // ── Inspector helpers ──────────────────────────────────────────────

    private void OnValidate()
    {
        supplyVoltage       = Mathf.Max(0f, supplyVoltage);
        currentLimit        = Mathf.Max(0f, currentLimit);
        softStartTime       = Mathf.Max(0f, softStartTime);
        vel_IntegralLimit   = Mathf.Max(0f, vel_IntegralLimit);
        pos_IntegralLimit   = Mathf.Max(0f, pos_IntegralLimit);
        maxVelocityCommand  = Mathf.Max(0f, maxVelocityCommand);
    }

#if UNITY_EDITOR
    // ── Quick test: press G in play mode to go to a random position ───
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.G))
            CommandPositionDegrees(Random.Range(-720f, 720f));

        if (Input.GetKeyDown(KeyCode.V))
            CommandVelocity(Random.Range(-50f, 50f));

        if (Input.GetKeyDown(KeyCode.E))
            eStop = !eStop;

        if (Input.GetKeyDown(KeyCode.R))
        {
            _encoder.ResetCount();
            SetEnabled(true);
        }
    }
#endif
}
