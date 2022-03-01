using UnityEngine;
using System.Collections.Generic;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision

	float Un = 0.5f;
	float Ut = 0.8f;
	Vector3 g = new Vector3(0, -9.98f, 0);
	Mesh mesh;
    List<Vector3> touchedPoints = new List<Vector3>();
	Vector3[] vertices;
	// Use this for initialization
	void Start () 
	{
		Application.targetFrameRate = 120;
		mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;
		Vector3 forcePoint = mesh.vertices[0];
		Debug.Log(forcePoint);
		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
        I_ref[3, 3] = 1;
		//ball.transform.localPosition = forcePoint;
        Matrix4x4 R0 = Matrix4x4.Rotate(transform.rotation);
		Vector4 point1 = new Vector4(forcePoint.x,forcePoint.y,forcePoint.z,1);
		point1 = R0 * point1;
		forcePoint = new Vector3(point1.x,point1.y,point1.z);
		Vector3 FFF = new Vector3(0, 0, 10000);
		Vector3 Torque = Vector3.Cross(forcePoint , FFF);
		Vector4 Torque4 = new Vector4(Torque.x, Torque.y, Torque.z, 1);
		//Matrix4x4 Inertia = R0 * I_ref * Matrix4x4.Transpose(R0);
		//Debug.Log(Inertia);
		Matrix4x4 InverseI = GetInverseI();
        w = InverseI * Torque4;
        w *= 0.15f;
        //Debug.Log(R0);
    }

    Matrix4x4 GetInverseI()
	{
		Matrix4x4 R0 = Matrix4x4.Rotate(transform.rotation);
		Matrix4x4 Inertia = R0 * I_ref * Matrix4x4.Transpose(R0);
		Matrix4x4 InverseI = Matrix4x4.Inverse(Inertia);
		return InverseI;
	}
	Quaternion qAdd(Quaternion q1, Quaternion q2)
	{
		Quaternion q = new Quaternion();
		q.x = q1.x + q2.x;
		q.y = q1.y + q2.y;
		q.z = q1.z + q2.z;
		q.w = q1.w + q2.w;
		return q;
	}

	Matrix4x4 mAdd(Matrix4x4 m1, Matrix4x4 m2)
	{
		Matrix4x4 newMat = Matrix4x4.identity;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				newMat[i,j] = m1[i,j] + m2[i,j];
			}
		}
		return newMat;
	}

    Matrix4x4 mMinus(Matrix4x4 m1, Matrix4x4 m2)
    {
        Matrix4x4 newMat = Matrix4x4.identity;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                newMat[i, j] = m1[i, j] - m2[i, j];
            }
        }
        return newMat;
    }

	Matrix4x4 mMultiply(Matrix4x4 m1, float f)
    {
        Matrix4x4 newMat = Matrix4x4.identity;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                newMat[i, j] = m1[i, j] * f;
            }
        }
        return newMat;
    }
    Vector3 GetRx(Vector3 pointX, Matrix4x4 R0)
	{
		Vector3 result = new Vector3();
		//R0 = Matrix4x4.Rotate(transform.rotation);
        Vector4 point1 = new Vector4(pointX.x, pointX.y, pointX.z, 1);
        point1 = R0 * point1;
		result = new Vector3(point1.x, point1.y, point1.z);
        return result;
	}
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		for (int i = 0; i < vertices.Length; i+=10)
		{
			Vector3 testVertex = vertices[i];
			testVertex = transform.position + GetRx(testVertex, Matrix4x4.Rotate(transform.rotation));
			float PhiVertex = Vector3.Dot((testVertex - P), N);
			if (PhiVertex < 0.01)
				touchedPoints.Add(vertices[i]);
		}
		if (touchedPoints.Count != 0)
		{
		//	launched = false;
			Debug.Log(touchedPoints.Count);

            //Process collision
            Vector3 averagePoint = new Vector3();
            for (int i = 0; i < touchedPoints.Count; i++)
            {
                averagePoint += touchedPoints[i];
            }
            averagePoint /= touchedPoints.Count;
            touchedPoints.Clear();

            Vector3 Rxri = GetRx(averagePoint, Matrix4x4.Rotate(transform.rotation));
			Vector3 averagePointPosition = transform.position + Rxri;
            float averagePhiX = Vector3.Dot((averagePointPosition - P),N);
			transform.position -= (averagePhiX * N);

			if (Vector3.Dot(v, N) < 0)
			{
				Vector3 Vn = Vector3.Dot(v, N) * N;
				Vector3 Vt = v - Vn;
				Vector3 VNewN = -Un * Vn;

				float a = 1 - Ut * (1 + Un) * Vn.magnitude / Vt.magnitude;
				if (a < 0)
					a = 0;
				Vector3 VNewt = a * Vt;
				Debug.Log("a=" + a);
				Vector3 VNew = VNewt + VNewN;
				if (VNew == Vector3.zero)
					return;
				Matrix4x4 MatRxri = Get_Cross_Matrix(Rxri);
				Matrix4x4 K = mMinus(mMultiply(Matrix4x4.identity,1/mass) ,MatRxri * GetInverseI() * MatRxri);
				//Debug.Log(K);
				Vector3 J = Matrix4x4.Inverse(K) * (VNew - v);
				v = v + J / mass;
                w = w + GetRx(Vector3.Cross(Rxri, J), GetInverseI());
				w *= 0.9f;
            }
            //Debug.Log(averagePoint);
		//	v = Vector3.zero;
        }

	
		//Debug.Log(PhiVertex);
    }

    // Update is called once per frame
    void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
            v = new Vector3(10, 0, 0);
            launched =true;
		}

        if (!launched)
            return;
        // Part I: Update velocities
        dt = Time.deltaTime;
		Vector3 gravity = g * mass;
		Vector3 F = new Vector3();
		F += gravity;
		Vector3 a = F / mass;
		v += a * dt;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

        // Part III: Update position & orientation
        //Update linear status
        Vector3 x    = transform.position;

		x += v * dt;
		//Update angular status
		Quaternion q = transform.rotation;
		Vector3 w1 = w * dt / 2;
        w *= 0.99f;
        Quaternion q1 = new Quaternion(w1.x, w1.y, w1.z, 0);
		q1 = q1 * q;
		q = qAdd(q,q1);
        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = q;
	}
}
