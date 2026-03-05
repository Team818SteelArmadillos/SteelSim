using UnityEngine;

/// <summary>
/// Quadrature Encoder Simulation
///
/// Simulates a physical incremental rotary encoder attached to a DCMotor.
/// Produces two phase signals (A and B), a once-per-revolution index pulse (Z),
/// and maintains a signed pulse count — just like a real encoder IC would.
///
/// Wiring convention:
///   - Channel A leads B by 90° when rotating in the POSITIVE direction.
///   - Channel B leads A by 90° when rotating in the NEGATIVE direction.
///   - Index pulse Z fires once per revolution at the encoder's home angle.
///
/// Noise / resolution features:
///   - Configurable pulses-per-revolution (PPR).
///   - Optional Gaussian velocity noise to mimic bearing / signal jitter.
///   - Optional missing-pulse simulation (dropout rate).
/// </summary>
[RequireComponent(typeof(DCMotor))]
public class MotorEncoder : MonoBehaviour
{
    // ── Configuration ──────────────────────────────────────────────────

    [Header("Encoder Specification")]
    [Tooltip("Pulses Per Revolution (lines on the disk). Common: 100, 512, 1024, 2048.")]
    public int pulsesPerRevolution = 1000;

    [Tooltip("Simulate electrical noise on the phase signals.")]
    public bool simulateNoise = false;

    [Tooltip("Standard deviation of velocity noise in rad/s (only used if simulateNoise = true).")]
    public float velocityNoiseStdDev = 0.05f;

    [Tooltip("Probability [0–1] that a single edge is dropped (only if simulateNoise = true).")]
    [Range(0f, 0.05f)]
    public float dropoutRate = 0f;

    [Header("Index Pulse")]
    [Tooltip("Angle (radians) at which the Z index pulse fires each revolution.")]
    public float indexAngleRad = 0f;

    [Tooltip("Width of the Z pulse in encoder counts (1 = single-edge width).")]
    public int indexPulseWidthCounts = 1;

    // ── Runtime State — Read Only ──────────────────────────────────────

    [Header("Runtime State — Read Only")]
    [SerializeField] private long  _count;          // Signed accumulated pulse count
    [SerializeField] private bool  _channelA;       // Current A signal level
    [SerializeField] private bool  _channelB;       // Current B signal level
    [SerializeField] private bool  _indexPulse;     // Z signal
    [SerializeField] private float _measuredRPM;    // Velocity derived from count rate
    [SerializeField] private float _measuredAngle;  // Angle derived from count (degrees)

    // Internal
    private DCMotor _motor;
    private float   _subStepAccum;     // Fractional pulse accumulator
    private long    _lastCount;        // For velocity estimation
    private float   _velocityTimer;
    private const float VelocityUpdateInterval = 0.05f; // 50 ms velocity window

    // Quadrature state machine: 4 states per full cycle
    // State: 0=(A=0,B=0), 1=(A=1,B=0), 2=(A=1,B=1), 3=(A=0,B=1)
    private int _quadState;

    // ── Public Accessors ───────────────────────────────────────────────
    public long  Count          => _count;
    public bool  ChannelA       => _channelA;
    public bool  ChannelB       => _channelB;
    public bool  IndexPulse     => _indexPulse;
    public float MeasuredRPM    => _measuredRPM;
    public float MeasuredAngle  => _measuredAngle;

    /// <summary>Resets the pulse count to zero (like an index latch or software reset).</summary>
    public void ResetCount() => _count = 0;

    /// <summary>Returns position in degrees based purely on encoder count.</summary>
    public float PositionDegrees =>
        (float)_count / pulsesPerRevolution * 360f;

    /// <summary>Returns position in radians based purely on encoder count.</summary>
    public float PositionRadians =>
        (float)_count / pulsesPerRevolution * 2f * Mathf.PI;

    // ── Initialisation ─────────────────────────────────────────────────
    private void Awake()
    {
        _motor = GetComponent<DCMotor>();
    }

    // ── Update ─────────────────────────────────────────────────────────
    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // --- 1. Get motor velocity (with optional noise) ---
        float omega = _motor.AngularVelocity;
        if (simulateNoise)
            omega += SampleGaussian(0f, velocityNoiseStdDev);

        // --- 2. Compute how many encoder edges occurred this timestep ---
        // Each PPR pulse = (2π / PPR) radians.  There are 4 edges per cycle (quadrature).
        float edgesPerRad = pulsesPerRevolution * 4f / (2f * Mathf.PI);
        float edgesThisStep = omega * dt * edgesPerRad;

        _subStepAccum += edgesThisStep;

        // --- 3. Advance the quadrature state machine by whole edges ---
        int wholePulses = (int)_subStepAccum;
        _subStepAccum -= wholePulses;

        int direction = (wholePulses >= 0) ? 1 : -1;
        int steps     = Mathf.Abs(wholePulses);

        for (int i = 0; i < steps; i++)
        {
            // Optional dropout simulation
            if (simulateNoise && dropoutRate > 0f && Random.value < dropoutRate)
                continue; // skip this edge

            _quadState = (_quadState + direction * 4 + 4) % 4; // wrap 0–3
            _count    += direction;
            UpdateChannels();
        }

        // --- 4. Index (Z) pulse ---
        // Fire Z when the motor's physical angle is within one count of the index angle
        float motorAngleMod = Mathf.Repeat(_motor.AngleRad - indexAngleRad, 2f * Mathf.PI);
        float countsFromIndex = motorAngleMod / (2f * Mathf.PI) * pulsesPerRevolution;
        _indexPulse = countsFromIndex < indexPulseWidthCounts;

        // --- 5. Velocity estimation (Δcount / Δt) ---
        _velocityTimer += dt;
        if (_velocityTimer >= VelocityUpdateInterval)
        {
            long delta = _count - _lastCount;
            float revPerSec = (float)delta / pulsesPerRevolution / _velocityTimer;
            _measuredRPM   = revPerSec * 60f;
            _lastCount     = _count;
            _velocityTimer = 0f;
        }

        // --- 6. Measured angle (unwrapped) ---
        _measuredAngle = PositionDegrees;
    }

    // ── Helpers ────────────────────────────────────────────────────────

    private void UpdateChannels()
    {
        // Gray code quadrature pattern
        switch (_quadState)
        {
            case 0: _channelA = false; _channelB = false; break;
            case 1: _channelA = true;  _channelB = false; break;
            case 2: _channelA = true;  _channelB = true;  break;
            case 3: _channelA = false; _channelB = true;  break;
        }
    }

    /// <summary>Box-Muller Gaussian sample.</summary>
    private static float SampleGaussian(float mean, float stdDev)
    {
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        float z  = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + stdDev * z;
    }

    // ── Gizmo ──────────────────────────────────────────────────────────
    private void OnDrawGizmosSelected()
    {
        if (_motor == null) return;
        // Draw index position as a yellow line
        Gizmos.color = Color.yellow;
        Vector3 p = transform.position;
        Gizmos.DrawLine(p, p + new Vector3(
            Mathf.Cos(indexAngleRad), 0f, Mathf.Sin(indexAngleRad)) * 0.4f);
    }
}
