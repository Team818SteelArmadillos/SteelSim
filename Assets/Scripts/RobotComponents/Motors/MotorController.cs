using UnityEngine;

public class MotorController : MonoBehaviour
{
    [SerializeField] GameObject motorHousing;
    DCMotor motor;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        motor = motorHousing.GetComponent<DCMotor>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        
    }
}
