using UnityEngine;

/// <summary>
/// MotorShaftPhysics
///
/// Bridges the DCMotor simulation and Unity's physics engine.
/// Instead of directly setting rotation, this script applies torque to a
/// Rigidbody each FixedUpdate, so the shaft object fully participates in
/// Unity physics — it can collide, drive joints, push other rigidbodies, etc.
///
/// How it works:
///   The DCMotor produces a shaft torque (N·m). This script applies that torque
///   to the connected Rigidbody via Rigidbody.AddTorque(). It also reads back
///   the Rigidbody's actual angular velocity and feeds it into the DCMotor as
///   the true mechanical speed, so the motor's back-EMF and current are accurate.
///
///   This two-way coupling means:
///     • If a collision slows the shaft, the motor sees increased load → more current.
///     • If the motor is stalled, current spikes realistically (up to V/R).
///     • Joints, hinges, and constraints all resist the motor naturally.
///
/// Setup:
///   1. Add this script to the same GameObject as DCMotor / MotorEncoder / MotorController.
///   2. Assign a Rigidbody (on a child or any scene object) to Shaft Rigidbody.
///   3. Set Shaft Axis to the world or local axis the shaft should spin around.
///   4. The Rigidbody's own mass and drag become the mechanical load automatically.
///
/// Rigidbody recommendations:
///   • Set Angular Drag to 0 — the DCMotor handles all friction internally.
///   • Set Collision Detection to Continuous if the shaft spins fast.
///   • Freeze the position axes you don't want to move (e.g. freeze X/Z position
///     for a wheel axle so it doesn't fly off).
/// </summary>
[RequireComponent(typeof(DCMotor))]
public class MotorShaftPhysics : MonoBehaviour
{
    [Header("Shaft Rigidbody")]
    [Tooltip("The Rigidbody to apply motor torque to. Must be assigned.")]
    public Rigidbody shaftRigidbody;

    [Header("Shaft Axis")]
    [Tooltip("The world-space axis the motor torque is applied around.")]
    public Vector3 torqueAxis = Vector3.up;

    [Tooltip("Flip the torque direction if the shaft spins the wrong way.")]
    public bool invertDirection = false;

    [Header("Torque Application")]
    [Tooltip("ForceMode for AddTorque. Use Force for realistic inertia, or Acceleration to ignore Rigidbody mass.")]
    public ForceMode forceMode = ForceMode.Force;

    [Tooltip("Scale applied to the motor's output torque before passing to the Rigidbody. "
           + "Use this to account for gearbox ratio (e.g. 10 = 10:1 reduction).")]
    public float gearRatio = 1f;

    [Header("Back-Coupling")]
    [Tooltip("Feed the Rigidbody's real angular velocity back into the DCMotor. "
           + "Keep this ON for accurate current/back-EMF simulation. "
           + "Turn OFF if the Rigidbody has additional motion sources (e.g. vehicle chassis).")]
    public bool feedbackAngularVelocity = true;

    [Header("Load Feedback")]
    [Tooltip("Automatically estimate load torque from the Rigidbody's angular deceleration "
           + "and feed it into the DCMotor. Improves current accuracy under impact loads.")]
    public bool estimateLoadTorque = true;

    [Header("Runtime Info — Read Only")]
    [SerializeField] private float _appliedTorque;
    [SerializeField] private float _shaftRPM;
    [SerializeField] private float _estimatedLoad;

    private DCMotor _motor;
    private float   _prevAngularVel;
    private float   _trueAngleRad;      // Integrated from Rigidbody angular velocity — ground truth for encoder

    // ── Accessors ──────────────────────────────────────────────────────
    public float AppliedTorque   => _appliedTorque;
    public float ShaftRPM        => _shaftRPM;
    public float EstimatedLoad   => _estimatedLoad;

    // ── Lifecycle ──────────────────────────────────────────────────────

    private void Awake()
    {
        _motor = GetComponent<DCMotor>();

        if (shaftRigidbody == null)
        {
            // Try to find one on a child object automatically
            shaftRigidbody = GetComponentInChildren<Rigidbody>();
            if (shaftRigidbody != null)
                Debug.Log($"[MotorShaftPhysics] Auto-assigned Rigidbody on '{shaftRigidbody.name}'.");
            else
                Debug.LogError("[MotorShaftPhysics] No Rigidbody assigned or found on children. "
                             + "Please assign one in the Inspector.");
        }
    }

    private void FixedUpdate()
    {
        if (shaftRigidbody == null || _motor == null) return;

        float dt = Time.fixedDeltaTime;

        // ── 1. Read shaft angular velocity from Rigidbody ─────────────
        // Project the Rigidbody's angular velocity onto our shaft axis
        // to get the scalar speed the motor "sees".
        Vector3 axisNorm       = torqueAxis.normalized;
        float   rbAngularSpeed = Vector3.Dot(shaftRigidbody.angularVelocity, axisNorm);
        if (invertDirection) rbAngularSpeed = -rbAngularSpeed;

        _shaftRPM = rbAngularSpeed * 9.5492f;

        // ── 2. Feed real shaft speed AND angle back into the motor model ──
        // This keeps the encoder reading the Rigidbody's true physical state.
        // Without this, DCMotor integrates _angle independently and the encoder
        // drifts away from the real position the moment any external force acts.
        if (feedbackAngularVelocity)
        {
            // Integrate true cumulative angle from Rigidbody angular velocity.
            // We use the same simple Euler here because this is just bookkeeping
            // (accumulating a real measured signal), not a physics simulation.
            _trueAngleRad += rbAngularSpeed * dt;

            OverrideMotorAngularVelocity(rbAngularSpeed);
            _motor.OverrideAngle(_trueAngleRad);
        }

        // ── 3. Estimate load torque from angular deceleration ─────────
        if (estimateLoadTorque)
        {
            float dOmega = rbAngularSpeed - _prevAngularVel;
            float alpha  = (dt > 0f) ? dOmega / dt : 0f;
            // τ_load = J_rb * α  (Rigidbody inertia tensor projected onto axis)
            float rbInertia    = ProjectedInertia(shaftRigidbody, axisNorm);
            float motorTorque  = _motor.Torque;
            _estimatedLoad     = Mathf.Max(0f, motorTorque - rbInertia * alpha);
            _motor.SetLoadTorque(_estimatedLoad);
        }

        _prevAngularVel = rbAngularSpeed;

        // ── 4. Apply motor torque to the Rigidbody ────────────────────
        float outputTorque = _motor.Torque * gearRatio;
        if (invertDirection) outputTorque = -outputTorque;

        _appliedTorque = outputTorque;
        shaftRigidbody.AddTorque(axisNorm * outputTorque, forceMode);
    }

    // ── Helpers ────────────────────────────────────────────────────────

    /// <summary>
    /// Patches the DCMotor's angular velocity to match the Rigidbody ground truth.
    /// Uses reflection-free access via a dedicated setter on DCMotor.
    /// </summary>
    private void OverrideMotorAngularVelocity(float omega)
    {
        _motor.OverrideAngularVelocity(omega);
    }

    /// <summary>
    /// Returns the moment of inertia of the Rigidbody projected onto a given axis.
    /// Unity stores the inertia tensor in local principal axes; we rotate it to world.
    /// </summary>
    private static float ProjectedInertia(Rigidbody rb, Vector3 worldAxis)
    {
        // Transform the world axis into the Rigidbody's inertia-tensor space
        Vector3 localAxis = Quaternion.Inverse(rb.inertiaTensorRotation)
                          * Quaternion.Inverse(rb.transform.rotation)
                          * worldAxis.normalized;

        Vector3 I = rb.inertiaTensor; // principal moments (x, y, z)
        return localAxis.x * localAxis.x * I.x
             + localAxis.y * localAxis.y * I.y
             + localAxis.z * localAxis.z * I.z;
    }

    // ── Gizmo ──────────────────────────────────────────────────────────
    private void OnDrawGizmosSelected()
    {
        Vector3 origin = shaftRigidbody != null
                       ? shaftRigidbody.transform.position
                       : transform.position;
        Vector3 axis = torqueAxis.normalized;

        // Draw shaft axis
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(origin - axis * 0.5f, origin + axis * 0.5f);
        Gizmos.DrawWireSphere(origin + axis * 0.5f, 0.03f);

        // Draw torque arc in the plane perpendicular to the axis
        Gizmos.color = Application.isPlaying
                     ? (_appliedTorque >= 0f ? Color.green : Color.red)
                     : Color.yellow;

        Vector3 perp = Vector3.Cross(axis, axis == Vector3.up ? Vector3.forward : Vector3.up).normalized;
        for (int i = 0; i < 16; i++)
        {
            float a0 = i       / 16f * Mathf.PI * 1.5f;
            float a1 = (i + 1) / 16f * Mathf.PI * 1.5f;
            Vector3 p0 = origin + (Mathf.Cos(a0) * perp + Mathf.Sin(a0) * Vector3.Cross(axis, perp)) * 0.25f;
            Vector3 p1 = origin + (Mathf.Cos(a1) * perp + Mathf.Sin(a1) * Vector3.Cross(axis, perp)) * 0.25f;
            Gizmos.DrawLine(p0, p1);
        }
    }
}
