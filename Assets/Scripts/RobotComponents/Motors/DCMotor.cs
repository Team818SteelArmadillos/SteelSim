using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DCMotor : MonoBehaviour
{
    [Header("Electrical Parameters")]
    public float R = 0.2f;          // Armature resistance (Ohms)
    public float L = 0.001f;        // Inductance (H)
    public float k_t = 0.02f;       // Torque constant (Nm/A)
    public float k_e = 0.02f;       // Back EMF constant (V/(rad/s))
    public float I_max = 100f;      // Current limit (A)

    [Header("Mechanical Parameters")]
    public float viscousFriction = 0.0001f;  // b (Nm per rad/s)
    public float coulombFriction = 0.001f;   // Static friction (Nm)
    public float rotorInertia = 1e-5f;       // Motor rotor inertia (kg*m^2)

    [Header("Gearbox")]
    public float gearRatio = 10f;  // Motor rotations per output rotation

    [Header("Power")]
    public float batteryVoltage = 12f;
    [Range(-1f, 1f)]
    public float dutyCycle = 0f;   // PWM input (-1 to 1)

    [Header("References")]
    [SerializeField] public Rigidbody outputBody;   // The driven rigidbody
    [SerializeField] public Vector3 motorAxis = Vector3.left;

    // Internal state
    private float current = 0f;
    private float motorSpeed = 0f;  // rad/s

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1️⃣ Measure output angular velocity in motor axis
        Vector3 localOmega = transform.InverseTransformDirection(outputBody.angularVelocity);
        float omega_out = Vector3.Dot(localOmega, motorAxis.normalized);

        // Convert to motor shaft speed
        motorSpeed = omega_out * gearRatio;

        // 2️⃣ Electrical dynamics
        float appliedVoltage = dutyCycle * batteryVoltage;
        float backEMF = k_e * motorSpeed;

        float dI = (appliedVoltage - R * current - backEMF) / L;
        current += dI * dt;

        // Current limiting
        current = Mathf.Clamp(current, -I_max, I_max);

        // 3️⃣ Motor torque at shaft
        float motorTorque = k_t * current;

        // 4️⃣ Friction losses
        float viscous = viscousFriction * motorSpeed;
        float coulomb = coulombFriction * Mathf.Sign(motorSpeed);

        float netMotorTorque = motorTorque - viscous - coulomb;

        // 5️⃣ Convert to output torque through gearbox
        float outputTorque = netMotorTorque * gearRatio;

        // 6️⃣ Apply torque to rigidbody
        Vector3 worldAxis = transform.TransformDirection(motorAxis.normalized);
        outputBody.AddTorque(worldAxis * outputTorque, ForceMode.Force);
    }

    // Optional getters for telemetry
    public float GetCurrent() => current;
    public float GetMotorSpeed() => motorSpeed;
}