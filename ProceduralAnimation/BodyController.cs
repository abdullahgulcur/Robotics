using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyController : MonoBehaviour
{
    public GameObject body;
    public float bodySpeed;
    public float bodyHeight;

    public GameObject[] endEffectorList;
    public GameObject[] legList;

    private bool canBodyMove = false;

    void Update()
    {
        //body.transform.Translate(Vector3.forward * Time.deltaTime * bodySpeed);

        if(canBodyMove)
            body.transform.Translate(Vector3.forward * Time.deltaTime * bodySpeed);

        BodyMovementControl();
        ControlBodyHeight();
    }

    private void BodyMovementControl()
    {
        //foreach(HexapodLegIK hlIK in legList.get)

        canBodyMove = true;

        foreach(GameObject leg in legList)
        {
            HexapodLegIK hlIK = leg.transform.GetComponent<HexapodLegIK>();
            if (!hlIK.canBodyMove)
            {
                canBodyMove = false;
                break;
            }
        }

        //if(!canBodyMove)
        //    body.transform.GetComponent<Rigidbody>().velocity = Vector3.zero;
    }

    private void ControlBodyHeight()
    {
        float totalHeight = 0;
        
        foreach (GameObject endEffector in endEffectorList)
            totalHeight += endEffector.transform.position.y;

        float averageHeight = totalHeight / endEffectorList.Length;

        body.transform.position = new Vector3(body.transform.position.x, averageHeight + bodyHeight, body.transform.position.z);

    }
}
