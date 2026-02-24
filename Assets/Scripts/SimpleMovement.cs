using UnityEngine;
using UnityEngine.InputSystem;

public class SimpleMovement : MonoBehaviour
{
    Rigidbody rb;
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

        Vector3 velocity = new Vector3(0f, rb.linearVelocity.y, 0f);
        if (Input.GetKey(KeyCode.W))
        {
            velocity.x += 10f;
        }
        if (Input.GetKey(KeyCode.S))
        {
            velocity.x -= 10f;
        }
        if (Input.GetKey(KeyCode.A))
        {
            velocity.z += 10f;
        }
        if (Input.GetKey(KeyCode.D))
        {
            velocity.z -= 10f;
        }
        Vector3 angularVelocity = new Vector3(0f, 0f, 0f);
        if (Input.GetKey(KeyCode.Q))
        {
            angularVelocity.y -= 4f;
        }
        if (Input.GetKey(KeyCode.E))
        {
            angularVelocity.y += 4f;
        }

        if (Input.GetMouseButtonDown(0))
        {
            GameObject newFuel = Instantiate(fuel);
            newFuel.GetComponent<Rigidbody>().position = fuelLauncher.GetComponent<Rigidbody>().position;
            newFuel.GetComponent<Rigidbody>().linearVelocity = transform.rotation * new Vector3(5f, 15f, 0);
        }
            rb.linearVelocity = velocity;
        rb.angularVelocity = angularVelocity;
    }

}
