using UnityEngine;
using UnityEngine.InputSystem;

public class SimpleMovement : MonoBehaviour
{
    Rigidbody rb;
    Vector3 velocity = new Vector3 (0f, 0f, 0f);
    GameObject fuelLauncher;
    [SerializeField] GameObject fuel;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        fuelLauncher = GameObject.FindGameObjectWithTag("FuelLauncher");
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.W))
        {
            velocity.x = 10f;
        }
        else
        {
            velocity = Vector3.zero;
        }

        if (Input.GetKeyDown(KeyCode.Q))
        {
            GameObject newFuel = Instantiate(fuel);
            newFuel.GetComponent<Rigidbody>().position = fuelLauncher.GetComponent<Rigidbody>().position;
            newFuel.GetComponent<Rigidbody>().linearVelocity = new Vector3(0, 10f, 10f);
        }
            rb.linearVelocity = velocity;
    }

}
