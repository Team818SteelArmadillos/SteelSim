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
/// Current protection — three configurable behaviours:
///
///   FOLDBACK (default) — When current exceeds the continuous limit, the output
///     voltage is scaled back proportionally to keep current under the peak limit.
///     The motor keeps running at reduced power.  This is how real motor drivers
///     (e.g. L298N, DRV8833) behave.
///
///   TRIP_AND_RETRY — Disables output for a configurable retry delay, then
///     automatically re-enables.  Good for simulating thermal cutouts.
///
///   HARD_LATCH — Permanently disables until SetEnabled(true) is called in code.
///     Use when you need to model a blown fuse or require explicit fault clearing.
///
/// Why the old default faulted at low speed:
///   Default params: V=12V, R=1Ω → stall current = V/R = 12A.
///   The old currentLimit default was 10A, so the motor faulted on the very first
///   frame before the shaft had moved at all.  The continuous limit is now 15A
///   (above stall current for the defaults) and the protection mode is Foldback,
///   so over-current reduces voltage rather than killing the motor entirely.
///
/// Features:
///   • Anti-windup (integrator clamping)
///   • Derivative-on-measurement (avoids derivative kick on setpoint change)
///   • Configurable output voltage limits (supply rail)
///   • Current-aware soft-start (slows ramp if current is already high)
///   • Runtime mode switching with bumpless transfer
///   • Emergency stop (E-Stop) that coasts or brakes
///   • Exposes all internal signals for oscilloscope / data-logging
/// </summary>
[RequireComponent(typeof(DCMotor))]
[RequireComponent(typeof(MotorEncoder))]
public class MotorController : MonoBehaviour
{
    // ── Enums ──────────────────────────────────────────────────────────

    public enum ControlMode     { Voltage, Velocity, Position }
    public enum StopMode        { Coast, ActiveBrake }
    public enum OverCurrentMode { Foldback, TripAndRetry, HardLatch }

    // ── Inspector ──────────────────────────────────────────────────────

    [Header("Control Mode")]
    public ControlMode mode = ControlMode.Velocity;

    [Header("Supply")]
    [Tooltip("Maximum supply voltage magnitude (V). Output is clamped to ±this value.")]
    public float supplyVoltage = 12f;

    [Header("Current Protection")]
    [Tooltip("Continuous current limit (A). Above this, foldback begins reducing voltage.\n"
           + "Set this to your motor's rated continuous current, or V/R (stall current) as a safe ceiling.")]
    public float continuousCurrentLimit = 15f;

    [Tooltip("Peak current limit (A). Hard ceiling used by all protection modes.\n"
           + "For Foldback: voltage is folded to zero at this value.\n"
           + "For TripAndRetry / HardLatch: a fault triggers if this is exceeded.")]
    public float peakCurrentLimit = 20f;

    [Tooltip("How the controller responds when current exceeds the peak limit.")]
    public OverCurrentMode overCurrentMode = OverCurrentMode.Foldback;

    [Tooltip("TripAndRetry only: seconds before automatically re-enabling after a trip.")]
    public float retryDelay = 1f;

    [Header("Soft-Start")]
    [Tooltip("Ramp time (seconds) from 0 → full voltage on enable.\n"
           + "The ramp also slows automatically if current is approaching the continuous limit,\n"
           + "preventing the ramp itself from causing an over-current trip on startup.")]
    public float softStartTime = 0.3f;

    [Header("Open-Loop Voltage Command")]
    [Tooltip("Voltage to apply when mode = Voltage.")]
    public float voltageSetpoint = 0f;

    [Header("Velocity PID  (rad/s)")]
    public float velocitySetpoint  = 0f;
    public float vel_Kp            = 2.0f;
    public float vel_Ki            = 5.0f;
    public float vel_Kd            = 0.01f;
    [Tooltip("Maximum integrator accumulation (V) — prevents windup.")]
    public float vel_IntegralLimit = 8f;

    [Header("Position PID  (rad)")]
    public float positionSetpoint  = 0f;
    public float pos_Kp            = 10.0f;
    public float pos_Ki            = 0.5f;
    public float pos_Kd            = 0.5f;
    [Tooltip("Maximum integrator accumulation (rad/s) — inner velocity limit.")]
    public float pos_IntegralLimit = 5f;
    [Tooltip("Clamps the velocity command issued by the position loop (rad/s).")]
    public float maxVelocityCommand = 20f;

    [Header("Emergency Stop")]
    public bool     eStop     = false;
    public StopMode eStopMode = StopMode.ActiveBrake;

    [Header("Runtime State — Read Only")]
    [SerializeField] private float _outputVoltage;
    [SerializeField] private float _velocityError;
    [SerializeField] private float _positionError;
    [SerializeField] private float _velIntegral;
    [SerializeField] private float _posIntegral;
    [SerializeField] private bool  _overCurrentFault;
    [SerializeField] private bool  _enabled = true;
    [SerializeField] private float _softStartFactor = 0f;
    [SerializeField] private float _foldbackFactor  = 1f;
    [SerializeField] private float _retryTimer;

    // ── Internal ───────────────────────────────────────────────────────

    private DCMotor      _motor;
    private MotorEncoder _encoder;
    private float        _prevMeasuredVelocity;
    private float        _prevMeasuredPosition;
    private float        _softStartTimer;
    private ControlMode  _lastMode;

    // ── Public API ─────────────────────────────────────────────────────

    public float OutputVoltage    => _outputVoltage;
    public bool  OverCurrentFault => _overCurrentFault;
    public bool  IsEnabled        => _enabled;

    /// <summary>Enable or disable the controller. Clears faults and restarts soft-start.</summary>
    public void SetEnabled(bool en)
    {
        if (en && !_enabled)
        {
            _softStartTimer   = 0f;
            _softStartFactor  = 0f;
            _overCurrentFault = false;
            _retryTimer       = 0f;
            _foldbackFactor   = 1f;
            ResetIntegrators();
        }
        _enabled = en;
    }

    public void CommandVoltage(float v)
    {
        mode = ControlMode.Voltage;
        voltageSetpoint = v;
    }

    public void CommandVelocity(float radPerSec)
    {
        mode = ControlMode.Velocity;
        velocitySetpoint = radPerSec;
    }

    public void CommandPosition(float radians)
    {
        mode = ControlMode.Position;
        positionSetpoint = radians;
    }

    public void CommandPositionDegrees(float degrees)
        => CommandPosition(degrees * Mathf.Deg2Rad);

    // ── Unity Lifecycle ────────────────────────────────────────────────

    private void Awake()
    {
        _motor    = GetComponent<DCMotor>();
        _encoder  = GetComponent<MotorEncoder>();
        _lastMode = mode;
    }

    private void FixedUpdate()
    {
        float dt         = Time.fixedDeltaTime;
        float absCurrent = Mathf.Abs(_motor.Current);

        // ── E-Stop ───────────────────────────────────────────────────
        if (eStop)
        {
            _outputVoltage = 0f;
            _motor.SetVoltage(eStopMode == StopMode.ActiveBrake ? 0f : _outputVoltage);
            return;
        }

        // ── Over-current protection ───────────────────────────────────
        HandleOverCurrent(absCurrent, dt);

        if (!_enabled)
        {
            _motor.SetVoltage(0f);
            return;
        }

        // ── Current-aware soft-start ──────────────────────────────────
        // The ramp timer advances at full speed when current is low, and slows
        // when current approaches the continuous limit — preventing the startup
        // ramp from itself triggering an over-current condition.
        float currentHeadroom = Mathf.Clamp01(
            1f - absCurrent / Mathf.Max(continuousCurrentLimit, 0.1f));
        float rampRate = softStartTime > 0f ? dt / softStartTime : 1f;
        _softStartTimer  += rampRate * currentHeadroom;
        _softStartFactor  = Mathf.Clamp01(_softStartTimer);

        // ── Bumpless transfer on mode change ──────────────────────────
        if (mode != _lastMode)
        {
            ResetIntegrators();
            _lastMode = mode;
        }

        // ── Control computation ───────────────────────────────────────
        float measuredVel = _encoder.MeasuredRPM / 9.5492f;
        float measuredPos = _encoder.PositionRadians;

        float command = mode switch
        {
            ControlMode.Voltage  => voltageSetpoint,
            ControlMode.Velocity => VelocityPID(velocitySetpoint, measuredVel, dt),
            ControlMode.Position => VelocityPID(
                Mathf.Clamp(PositionPID(positionSetpoint, measuredPos, dt),
                            -maxVelocityCommand, maxVelocityCommand),
                measuredVel, dt),
            _ => 0f
        };

        // ── Apply limits ──────────────────────────────────────────────
        command  = Mathf.Clamp(command, -supplyVoltage, supplyVoltage);
        command *= _softStartFactor;   // startup ramp
        command *= _foldbackFactor;    // over-current foldback

        _outputVoltage = command;
        _motor.SetVoltage(command);

        _prevMeasuredVelocity = measuredVel;
        _prevMeasuredPosition = measuredPos;
    }

    // ── Over-current handler ───────────────────────────────────────────

    private void HandleOverCurrent(float absCurrent, float dt)
    {
        switch (overCurrentMode)
        {
            case OverCurrentMode.Foldback:
            {
                // Proportionally reduce voltage as current climbs from continuous → peak.
                // Recovery is slow (5×/s), reduction is fast (50×/s) — protect first.
                if (absCurrent > continuousCurrentLimit)
                {
                    float band      = Mathf.Max(peakCurrentLimit - continuousCurrentLimit, 0.1f);
                    float overRatio = (absCurrent - continuousCurrentLimit) / band;
                    float target    = Mathf.Clamp01(1f - overRatio);
                    _foldbackFactor = Mathf.MoveTowards(_foldbackFactor, target,
                                        (target < _foldbackFactor ? 50f : 5f) * dt);
                }
                else
                {
                    _foldbackFactor = Mathf.MoveTowards(_foldbackFactor, 1f, 5f * dt);
                }

                // If we've somehow still blown past the peak, cut immediately
                if (absCurrent > peakCurrentLimit)
                    _foldbackFactor = 0f;

                _overCurrentFault = absCurrent > peakCurrentLimit;
                break;
            }

            case OverCurrentMode.TripAndRetry:
            {
                if (absCurrent > peakCurrentLimit && _enabled)
                {
                    _overCurrentFault = true;
                    _enabled          = false;
                    _retryTimer       = 0f;
                }
                if (!_enabled && _overCurrentFault)
                {
                    _retryTimer += dt;
                    if (_retryTimer >= retryDelay)
                        SetEnabled(true);
                }
                break;
            }

            case OverCurrentMode.HardLatch:
            {
                if (absCurrent > peakCurrentLimit)
                {
                    _overCurrentFault = true;
                    _enabled          = false;
                }
                break;
            }
        }
    }

    // ── PID Loops ──────────────────────────────────────────────────────

    private float VelocityPID(float setpoint, float measured, float dt)
    {
        _velocityError = setpoint - measured;
        _velIntegral   = Mathf.Clamp(_velIntegral + _velocityError * dt,
                                     -vel_IntegralLimit, vel_IntegralLimit);
        float derivative = dt > 0f ? -(measured - _prevMeasuredVelocity) / dt : 0f;
        return vel_Kp * _velocityError + vel_Ki * _velIntegral + vel_Kd * derivative;
    }

    private float PositionPID(float setpoint, float measured, float dt)
    {
        _positionError = setpoint - measured;
        _posIntegral   = Mathf.Clamp(_posIntegral + _positionError * dt,
                                     -pos_IntegralLimit, pos_IntegralLimit);
        float derivative = dt > 0f ? -(measured - _prevMeasuredPosition) / dt : 0f;
        return pos_Kp * _positionError + pos_Ki * _posIntegral + pos_Kd * derivative;
    }

    private void ResetIntegrators()
    {
        _velIntegral = 0f;
        _posIntegral = 0f;
    }

    // ── Inspector validation ───────────────────────────────────────────

    private void OnValidate()
    {
        supplyVoltage          = Mathf.Max(0f, supplyVoltage);
        continuousCurrentLimit = Mathf.Max(0f, continuousCurrentLimit);
        peakCurrentLimit       = Mathf.Max(continuousCurrentLimit, peakCurrentLimit);
        softStartTime          = Mathf.Max(0f, softStartTime);
        retryDelay             = Mathf.Max(0f, retryDelay);
        vel_IntegralLimit      = Mathf.Max(0f, vel_IntegralLimit);
        pos_IntegralLimit      = Mathf.Max(0f, pos_IntegralLimit);
        maxVelocityCommand     = Mathf.Max(0f, maxVelocityCommand);
    }

#if UNITY_EDITOR
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
