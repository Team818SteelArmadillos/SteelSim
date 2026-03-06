using UnityEngine;

/// <summary>
/// MotorShaftPhysics
///
/// Bridges the DCMotor simulation and Unity's physics engine.
/// Applies motor torque to a Rigidbody each FixedUpdate so the shaft
/// participates in Unity physics (collisions, joints, etc.), while locking
/// its position and all non-spin rotation axes back to the initial transform
/// every frame.
///
/// Constraint strategy:
///   On Start, the shaft's world position and world rotation are recorded.
///   Every FixedUpdate, after applying torque:
///     • Position  → always restored to the initial world position.
///     • Rotation  → reconstructed as:
///                     initialRotation × Quaternion.AngleAxis(spinAngle, localAxis)
///                   This means the shaft can only ever rotate around its own
///                   spin axis relative to how it was placed in the scene.
///                   All other rotation axes are implicitly locked.
///     • Linear velocity  → zeroed (position is locked, so velocity must be too).
///     • Angular velocity → only the spin-axis component is kept; all others zeroed.
///
/// Two-way motor coupling:
///   • Motor → Rigidbody : AddTorque() applies the motor's output torque.
///   • Rigidbody → Motor : real angular speed and accumulated angle are fed back
///                         into DCMotor so back-EMF and current stay accurate
///                         even when collisions or load changes affect the shaft.
///
/// Setup:
///   1. Add to the same GameObject as DCMotor, MotorEncoder, MotorController.
///   2. Assign a Rigidbody (child or scene object) to Shaft Rigidbody.
///   3. Set Torque Axis to the LOCAL axis of the shaft to spin around (e.g. Vector3.up).
///   4. Set Angular Drag = 0 on the Rigidbody — DCMotor handles all friction.
///   5. You do NOT need to freeze any Rigidbody constraints — this script handles it.
/// </summary>
[RequireComponent(typeof(DCMotor))]
public class MotorShaftPhysics : MonoBehaviour
{
    [Header("Shaft Rigidbody")]
    [Tooltip("The Rigidbody to apply motor torque to.")]
    public Rigidbody shaftRigidbody;

    [Header("Shaft Axis")]
    [Tooltip("The LOCAL-space axis of the Shaft Rigidbody the motor spins around. "
           + "Vector3.up = local Y, Vector3.forward = local Z, Vector3.right = local X. "
           + "Rotates with the Rigidbody, so the axis stays correct when the motor is tilted.")]
    public Vector3 torqueAxis = Vector3.up;

    [Tooltip("Flip the spin direction if the shaft rotates the wrong way.")]
    public bool invertDirection = false;

    [Header("Torque Application")]
    [Tooltip("ForceMode for AddTorque. Force = realistic inertia. Acceleration = ignores Rigidbody mass.")]
    public ForceMode forceMode = ForceMode.Force;

    [Tooltip("Scale applied to the motor's output torque before passing to the Rigidbody. "
           + "Use this to account for gearbox ratio (e.g. 10 = 10:1 reduction).")]
    public float gearRatio = 1f;

    [Header("Back-Coupling")]
    [Tooltip("Feed the Rigidbody's real angular velocity back into the DCMotor. "
           + "Keep this ON for accurate current/back-EMF simulation. "
           + "Turn OFF if the Rigidbody has additional motion sources (e.g. vehicle chassis).")]
    public bool feedbackAngularVelocity = true;

    [Header("Axis Constraint")]
    [Tooltip("Zero out any angular velocity on axes other than the shaft axis every FixedUpdate. "
           + "This prevents physics solver drift and collision torques from tipping or wobbling the shaft. "
           + "Disable if you are using a HingeJoint — the joint already constrains the axes and "
           + "double-constraining will cause jitter.")]
    public bool constrainToShaftAxis = true;

    [Header("Load Feedback")]
    [Tooltip("Automatically estimate load torque from the Rigidbody's angular deceleration "
           + "and feed it into the DCMotor. Improves current accuracy under impact loads.")]
    public bool estimateLoadTorque = true;

    [Header("Initial Transform — Set at Runtime")]
    [SerializeField] private Vector3 _initialPosition;
    [SerializeField] private Quaternion _initialRotation;
    [SerializeField] private float _spinAngleDeg;

    [Header("Runtime Info — Read Only")]
    [SerializeField] private float _appliedTorque;
    [SerializeField] private float _shaftRPM;
    [SerializeField] private float _estimatedLoad;

    private DCMotor _motor;
    private float _prevAngularVel;
    private float _trueAngleRad;

    // ── Accessors ──────────────────────────────────────────────────────
    public float AppliedTorque => _appliedTorque;
    public float ShaftRPM => _shaftRPM;
    public float EstimatedLoad => _estimatedLoad;

    // ── Lifecycle ──────────────────────────────────────────────────────

    private void Awake()
    {
        _motor = GetComponent<DCMotor>();

        if (shaftRigidbody == null)
        {
            shaftRigidbody = GetComponentInChildren<Rigidbody>();
            if (shaftRigidbody != null)
                Debug.Log($"[MotorShaftPhysics] Auto-assigned Rigidbody on '{shaftRigidbody.name}'.");
            else
                Debug.LogError("[MotorShaftPhysics] No Rigidbody assigned or found on children. "
                             + "Please assign one in the Inspector.");
        }
    }

    private void Start()
    {
        if (shaftRigidbody == null) return;

        // Record the shaft's world-space pose at scene start.
        // Everything will be locked to this — only the spin angle changes.
        _initialPosition = shaftRigidbody.position;
        _initialRotation = shaftRigidbody.rotation;
        _spinAngleDeg = 0f;

        Debug.Log($"[MotorShaftPhysics] Initial position: {_initialPosition}, "
                + $"Initial rotation: {_initialRotation.eulerAngles}");
    }

    private void FixedUpdate()
    {
        if (shaftRigidbody == null || _motor == null) return;

        float dt = Time.fixedDeltaTime;

        // ── 1. Resolve spin axis in world space ───────────────────────
        // Use the INITIAL rotation to define the world-space spin axis,
        // not the current rotation — otherwise the axis would drift if
        // anything tried to perturb the shaft's orientation.
        Vector3 worldSpinAxis = _initialRotation * torqueAxis.normalized;

        // ── 2. Read shaft angular speed along the spin axis ───────────
        float rbAngularSpeed = Vector3.Dot(shaftRigidbody.angularVelocity, worldSpinAxis);
        if (invertDirection) rbAngularSpeed = -rbAngularSpeed;

        _shaftRPM = rbAngularSpeed * 9.5492f;

        // ── 3. Feed real speed and angle back into the motor model ─────
        if (feedbackAngularVelocity)
        {
            _trueAngleRad += rbAngularSpeed * dt;
            _motor.OverrideAngularVelocity(rbAngularSpeed);
            _motor.OverrideAngle(_trueAngleRad);
        }

        // ── 4. Estimate and feed load torque ──────────────────────────
        if (estimateLoadTorque)
        {
            float alpha = (dt > 0f) ? (rbAngularSpeed - _prevAngularVel) / dt : 0f;
            float rbInertia = ProjectedInertia(shaftRigidbody, worldSpinAxis);
            _estimatedLoad = Mathf.Max(0f, _motor.Torque - rbInertia * alpha);
            _motor.SetLoadTorque(_estimatedLoad);
        }

        _prevAngularVel = rbAngularSpeed;

        // ── 5. Constrain rotation to shaft axis only ──────────────────
        // Strip any angular velocity that has crept onto axes the shaft
        // should never rotate around (solver drift, collision impulses, etc).
        if (constrainToShaftAxis)
        {
            shaftRigidbody.angularVelocity = worldSpinAxis * rbAngularSpeed;
        }

        // ── 6. Apply motor torque ─────────────────────────────────────
        float outputTorque = _motor.Torque * gearRatio;
        if (invertDirection) outputTorque = -outputTorque;

        _appliedTorque = outputTorque;
        shaftRigidbody.AddTorque(worldSpinAxis * outputTorque, forceMode);

        // ── 7. Lock position and all non-spin rotation axes ───────────
        _spinAngleDeg = _motor.AngleDeg;
        if (invertDirection) _spinAngleDeg = -_spinAngleDeg;

        // Reconstruct rotation as: initial pose × pure spin around the local axis.
        // Any rotation that crept onto other axes this frame is discarded entirely.
        Quaternion spinOnly = Quaternion.AngleAxis(_spinAngleDeg, torqueAxis.normalized);
        shaftRigidbody.MoveRotation(_initialRotation * spinOnly);

        // Lock position — the shaft spins in place, it never translates.
        shaftRigidbody.MovePosition(_initialPosition);

        // Zero linear velocity — position is locked so any linear velocity is invalid.
        shaftRigidbody.linearVelocity = Vector3.zero;
    }

    // ── Helpers ────────────────────────────────────────────────────────

    private void OverrideMotorAngularVelocity(float omega)
    {
        _motor.OverrideAngularVelocity(omega);
    }

    private static float ProjectedInertia(Rigidbody rb, Vector3 worldAxis)
    {
        Vector3 localAxis = Quaternion.Inverse(rb.inertiaTensorRotation)
                          * Quaternion.Inverse(rb.transform.rotation)
                          * worldAxis.normalized;
        Vector3 I = rb.inertiaTensor;
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

        Quaternion baseRot = Application.isPlaying ? _initialRotation
                                                   : (shaftRigidbody != null
                                                       ? shaftRigidbody.rotation
                                                       : transform.rotation);
        Vector3 axis = baseRot * torqueAxis.normalized;

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(origin - axis * 0.5f, origin + axis * 0.5f);
        Gizmos.DrawWireSphere(origin + axis * 0.5f, 0.03f);

        if (Application.isPlaying)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(_initialPosition, 0.04f);
        }

        Gizmos.color = Application.isPlaying
                     ? (_appliedTorque >= 0f ? Color.green : Color.red)
                     : Color.yellow;

        Vector3 perp = Vector3.Cross(axis,
            Mathf.Abs(Vector3.Dot(axis, Vector3.up)) < 0.99f ? Vector3.up : Vector3.forward).normalized;

        for (int i = 0; i < 16; i++)
        {
            float a0 = i / 16f * Mathf.PI * 1.5f;
            float a1 = (i + 1) / 16f * Mathf.PI * 1.5f;
            Vector3 p0 = origin + (Mathf.Cos(a0) * perp + Mathf.Sin(a0) * Vector3.Cross(axis, perp)) * 0.25f;
            Vector3 p1 = origin + (Mathf.Cos(a1) * perp + Mathf.Sin(a1) * Vector3.Cross(axis, perp)) * 0.25f;
            Gizmos.DrawLine(p0, p1);
        }
    }
}