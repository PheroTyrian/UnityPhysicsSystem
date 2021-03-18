using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum CollisionType
{
    sphere,
    plane,
    cuboid
}

public class Physics : MonoBehaviour
{
    //Movement
    public Vector3 m_velocity;
    public Vector3 m_uniformAcceleration;
    public float m_gravityStrength = 0.0f;
    //Collision stuff
    public CollisionType m_collisionType = CollisionType.sphere;
    const float m_contactSpeed = 0.2f;
    public float m_sphereRadius;
    public float m_mass; //This says whether or not to apply impulse to this object
    public float m_restitution; //How much bounce

    // Start is called before the first frame update
    void Start()
    {
        var mesh = GetComponent<MeshFilter>();
        Vector3 scale = GetComponent<Transform>().localScale;
        //Set sphereRadius (fuzzy bounding area if shape isn't a sphere)
        if (m_collisionType == CollisionType.sphere)
        {
            m_sphereRadius = 0.5f * scale.x;
        }
        else //Find the most distant vertex from the centre of the shape and set the collision check radius to it
        {
            m_sphereRadius = 0;
            foreach (Vector3 vec in mesh.mesh.vertices)
            {
                Vector3 tempVec = new Vector3(vec.x * scale.x, vec.y * scale.y, vec.z * scale.z);
                if (tempVec.magnitude > m_sphereRadius)
                    m_sphereRadius = tempVec.magnitude;
            }
        }
    }

    //Little calculator for the 1-dimensional velocity of a projectile after a collision with another object
    float speedAfterCollision(float restitution, float vA, float vB, float massA, float massB)
    {
        if (massA != 0 && massB != 0)
        {
            float top = (massA * vA) + (massB * vB) + (massB * restitution * (vB - vA));
            return top / (massA + massB);
        }
        else if (massA != 0)
            return vB - restitution * (vA - vB);
        else
            return vA;
    }

    struct CollisionPoint
    {
        public bool occurred;
        public Vector3 position;
        public Vector3 normal;

        public CollisionPoint(bool col, Vector3 pos = new Vector3(), Vector3 norm = new Vector3())
        {
            occurred = col;
            position = pos;
            normal = norm;
        }
    }

    struct Plane
    {
        public Vector3 position;
        public Vector3 normal;
        public Vector3 halfWidth;
        public Vector3 halfHeight;

        public Plane(Vector3 pos, Vector3 norm, Vector3 w, Vector3 h)
        {
            position = pos;
            normal = norm;
            halfWidth = w;
            halfHeight = h;
        }
    }

    CollisionPoint findCPSphereOnLine(Vector3 spherePosition, float sphereRadius, Vector3 lineStart, Vector3 lineEnd)
    {
        Vector3 relativeSpherePos = spherePosition - lineStart;
        Vector3 line = lineEnd - lineStart;

        float closestPointOnLine = Vector3.Dot(relativeSpherePos, line.normalized);
        closestPointOnLine = Mathf.Clamp(closestPointOnLine, 0, line.magnitude);

        CollisionPoint collision = new CollisionPoint(false, lineStart + (closestPointOnLine * line.normalized));
        collision.normal = (spherePosition - collision.position).normalized;

        if ((spherePosition - collision.position).magnitude <= sphereRadius)
        {
            collision.occurred = true;
        }

        return collision;
    }
    
    CollisionPoint findCPSphereOnPlane(Vector3 spherePosition, float sphereRadius, Plane plane)
    {
        Vector3 spherePosFromPlane = spherePosition - plane.position;
        float distanceFromPlane = Vector3.Dot(spherePosFromPlane, plane.normal.normalized);
        if (Mathf.Abs(distanceFromPlane) <= sphereRadius) //Sphere close enough to infinite plane
        {
            float sphereWidthDist = Vector3.Dot(plane.halfWidth.normalized, spherePosFromPlane);
            float sphereHeightDist = Vector3.Dot(plane.halfHeight.normalized, spherePosFromPlane);

            if (Mathf.Abs(sphereWidthDist) <= plane.halfWidth.magnitude && Mathf.Abs(sphereHeightDist) <= plane.halfHeight.magnitude) //Collision on plane surface
            {
                Vector3 relativeCollisionPosition = spherePosFromPlane - (distanceFromPlane * plane.normal.normalized);
                return new CollisionPoint(true, plane.position + relativeCollisionPosition, plane.normal.normalized);
            }
            else if (Mathf.Abs(sphereWidthDist) / plane.halfWidth.magnitude > Mathf.Abs(sphereHeightDist) / plane.halfHeight.magnitude) //Sphere further width-wise than height-wise
            {
                if (sphereWidthDist > 0) //Sphere near +ve width edge
                {
                    return findCPSphereOnLine(spherePosition, sphereRadius, plane.position + plane.halfWidth + plane.halfHeight, plane.position + plane.halfWidth - plane.halfHeight);
                }
                else
                {
                    return findCPSphereOnLine(spherePosition, sphereRadius, plane.position - plane.halfWidth + plane.halfHeight, plane.position - plane.halfWidth - plane.halfHeight);
                }
            }
            else //Sphere further height-wise than width-wise
            {
                if (sphereHeightDist > 0) //Sphere near +ve height edge
                {
                    return findCPSphereOnLine(spherePosition, sphereRadius, plane.position + plane.halfWidth + plane.halfHeight, plane.position - plane.halfWidth + plane.halfHeight);
                }
                else
                {
                    return findCPSphereOnLine(spherePosition, sphereRadius, plane.position + plane.halfWidth - plane.halfHeight, plane.position - plane.halfWidth - plane.halfHeight);
                }
            }
        }
        else
            return new CollisionPoint(false);
    }

    /*Vector3 pointPosFromPlane = point - plane.position;
    float normalDist = Vector3.Dot(plane.normal.normalized, pointPosFromPlane);

        if (normalDist <= 0)
        {
            float pointWidthDist = Vector3.Dot(plane.halfWidth.normalized, pointPosFromPlane);
    float pointHeightDist = Vector3.Dot(plane.halfHeight.normalized, pointPosFromPlane);

            if (Mathf.Abs(pointWidthDist) <= plane.halfWidth.magnitude && Mathf.Abs(pointHeightDist) <= plane.halfHeight.magnitude)
            {
                CollisionPoint collision = new CollisionPoint(true);
    collision.position = point - (normalDist* plane.normal.normalized);
                collision.normal = plane.normal.normalized;
                return collision;
            }
        }
        
        return new CollisionPoint(false);*/

    /*CollisionPoint findCPLineOnPlane(Vector3 lineStart, Vector3 lineEnd, Plane plane)
    {
        Vector3 line = lineEnd - lineStart;
        float posDotN = Vector3.Dot(plane.normal.normalized, lineStart);
        float lineDotN = Vector3.Dot(plane.normal.normalized, line);

        if (lineDotN == 0) //Line parallel to plane
            return new CollisionPoint(false);

        float intersectionOnLine = (plane.position.magnitude - posDotN) / lineDotN;

        if (intersectionOnLine >= 0 && intersectionOnLine <= 1)
        {

        }

        //Vector3 intersection = lineStart + ((plane.position.magnitude - posDotN) / lineDotN) * line;
        //TODO: continue this
    }*/

    void sphereOnSphere(Transform transformA, Transform transformB, Physics physicsA, Physics physicsB)
    {
        Vector3 centreToCentre = transformB.position - transformA.position;
        //Are the objects moving towards each other?
        float vA = Vector3.Dot(physicsA.m_velocity, centreToCentre.normalized);
        float vB = Vector3.Dot(physicsB.m_velocity, centreToCentre.normalized);
        float netMovement = vA - vB;

        if (netMovement > m_contactSpeed) //Moving towards so collision should occur
        {
            float speedA = speedAfterCollision(physicsA.m_restitution * physicsB.m_restitution, vA, vB, physicsA.m_mass, physicsB.m_mass);
            physicsA.m_velocity += (speedA - vA) * centreToCentre.normalized;

            float speedB = speedAfterCollision(physicsA.m_restitution * physicsB.m_restitution, vB, vA, physicsB.m_mass, physicsA.m_mass);
            physicsB.m_velocity += (speedB - vB) * centreToCentre.normalized;
        }
        else if (netMovement > -m_contactSpeed / 2) //In contact
        {
            float relativeRadiusSize = physicsA.m_sphereRadius / (physicsA.m_sphereRadius + physicsB.m_sphereRadius);
            Vector3 collisionPoint = transformA.position + centreToCentre * relativeRadiusSize;
            transformA.position = collisionPoint - physicsA.m_sphereRadius * centreToCentre.normalized;
            transformB.position = collisionPoint + physicsB.m_sphereRadius * centreToCentre.normalized;

            if (physicsA.m_mass != 0 && physicsB.m_mass != 0)
            {
                physicsA.m_velocity -= netMovement * centreToCentre.normalized / 2;
                physicsB.m_velocity += netMovement * centreToCentre.normalized / 2;
            }
            else if (physicsA.m_mass != 0)
                physicsA.m_velocity -= netMovement * centreToCentre.normalized;
            else if (physicsB.m_mass != 0)
                physicsB.m_velocity += netMovement * centreToCentre.normalized;
        }
        //Else they're moving away from each other
    }

    void sphereOnPlane(Transform sphereTransform, Transform planeTransform, Physics spherePhysics, Physics planePhysics)
    {
        Matrix4x4 rotation = Matrix4x4.Rotate(Quaternion.Euler(planeTransform.eulerAngles));

        Vector3 planeNormal = rotation.MultiplyVector(new Vector3(0, 1, 0)).normalized;
        Vector3 planeX = rotation.MultiplyVector(new Vector3(1, 0, 0)).normalized * planeTransform.localScale.x * 5;
        Vector3 planeZ = rotation.MultiplyVector(new Vector3(0, 0, 1)).normalized * planeTransform.localScale.z * 5;

        Plane plane = new Plane(planeTransform.position, planeNormal, planeX, planeZ);
        CollisionPoint collision = findCPSphereOnPlane(sphereTransform.position, spherePhysics.m_sphereRadius, plane);

        if (!collision.occurred)
            return;

        Vector3 pointToCentre = sphereTransform.position - collision.position;

        float vSphere = Vector3.Dot(spherePhysics.m_velocity, pointToCentre.normalized);
        float vPlane = Vector3.Dot(planePhysics.m_velocity, pointToCentre.normalized);
        float netMovement = vPlane - vSphere;

        if (netMovement > m_contactSpeed) //Moving towards so collision should occur
        {
            float speedSphere = speedAfterCollision(spherePhysics.m_restitution * planePhysics.m_restitution, vSphere, vPlane, spherePhysics.m_mass, planePhysics.m_mass);
            spherePhysics.m_velocity += (speedSphere - vSphere) * pointToCentre.normalized;

            float speedPlane = speedAfterCollision(spherePhysics.m_restitution * planePhysics.m_restitution, vPlane, vSphere, planePhysics.m_mass, spherePhysics.m_mass);
            planePhysics.m_velocity += (speedPlane - vPlane) * pointToCentre.normalized;
        }
        else if (netMovement > -m_contactSpeed / 2) //In contact
        {
            if (spherePhysics.m_mass != 0)
            {
                spherePhysics.m_velocity += netMovement * pointToCentre.normalized;
                sphereTransform.position = pointToCentre.normalized * spherePhysics.m_sphereRadius + collision.position;
            }
        }
        //Else they're moving away from each other
    }

    void sphereOnCube(Transform sphereTransform, Transform cubeTransform, Physics spherePhysics, Physics cubePhysics)
    {
        //Rotate cube vectors into worldspace
        Matrix4x4 rotation = Matrix4x4.Rotate(Quaternion.Euler(cubeTransform.eulerAngles));

        Vector3 cubeX = rotation.MultiplyVector(new Vector3(1, 0, 0)).normalized * cubeTransform.localScale.x * 0.5f;
        Vector3 cubeY = rotation.MultiplyVector(new Vector3(0, 1, 0)).normalized * cubeTransform.localScale.y * 0.5f;
        Vector3 cubeZ = rotation.MultiplyVector(new Vector3(0, 0, 1)).normalized * cubeTransform.localScale.z * 0.5f;

        //Check if there's a collision against the closest side, and get the collision point
        Vector3 sphereRelativePosition = sphereTransform.position - cubeTransform.position;

        float distInX = Vector3.Dot(sphereRelativePosition, cubeX.normalized) / cubeX.magnitude;
        float distInY = Vector3.Dot(sphereRelativePosition, cubeY.normalized) / cubeY.magnitude;
        float distInZ = Vector3.Dot(sphereRelativePosition, cubeZ.normalized) / cubeZ.magnitude;

        CollisionPoint collision = new CollisionPoint(false);
        if (Mathf.Abs(distInX) > Mathf.Abs(distInY) && Mathf.Abs(distInX) > Mathf.Abs(distInZ)) //Closest to X sides
        {
            if (distInX > 0)
            {
                Plane plane = new Plane(cubeTransform.position + cubeX, cubeX, cubeY, cubeZ);
                collision = findCPSphereOnPlane(sphereTransform.position, spherePhysics.m_sphereRadius, plane);
            }
            else
            {
                Plane plane = new Plane(cubeTransform.position - cubeX, -cubeX, cubeY, cubeZ);
                collision = findCPSphereOnPlane(sphereTransform.position, spherePhysics.m_sphereRadius, plane);
            }
        }
        else if (Mathf.Abs(distInY) > Mathf.Abs(distInZ)) //Closest to Y sides
        {
            if (distInY > 0)
            {
                Plane plane = new Plane(cubeTransform.position + cubeY, cubeY, cubeX, cubeZ);
                collision = findCPSphereOnPlane(sphereTransform.position, spherePhysics.m_sphereRadius, plane);
            }
            else
            {
                Plane plane = new Plane(cubeTransform.position - cubeY, -cubeY, cubeX, cubeZ);
                collision = findCPSphereOnPlane(sphereTransform.position, spherePhysics.m_sphereRadius, plane);
            }
        }
        else //Closest to Z sides
        {
            if (distInZ > 0)
            {
                Plane plane = new Plane(cubeTransform.position + cubeZ, cubeZ, cubeY, cubeX);
                collision = findCPSphereOnPlane(sphereTransform.position, spherePhysics.m_sphereRadius, plane);
            }
            else
            {
                Plane plane = new Plane(cubeTransform.position - cubeZ, -cubeZ, cubeY, cubeX);
                collision = findCPSphereOnPlane(sphereTransform.position, spherePhysics.m_sphereRadius, plane);
            }
        }

        if (!collision.occurred)
            return;

        Vector3 pointToCentre = sphereTransform.position - collision.position;

        float vSphere = Vector3.Dot(spherePhysics.m_velocity, pointToCentre.normalized);
        float vCube = Vector3.Dot(cubePhysics.m_velocity, pointToCentre.normalized);
        float netMovement = vCube - vSphere;

        if (netMovement > m_contactSpeed) //Moving towards so collision should occur
        {
            float speedSphere = speedAfterCollision(spherePhysics.m_restitution * cubePhysics.m_restitution, vSphere, vCube, spherePhysics.m_mass, cubePhysics.m_mass);
            spherePhysics.m_velocity += (speedSphere - vSphere) * pointToCentre.normalized;

            float speedPlane = speedAfterCollision(spherePhysics.m_restitution * cubePhysics.m_restitution, vCube, vSphere, cubePhysics.m_mass, spherePhysics.m_mass);
            cubePhysics.m_velocity += (speedPlane - vCube) * pointToCentre.normalized;
        }
        else if (netMovement > -m_contactSpeed / 2) //In contact
        {
            sphereTransform.position = collision.position + spherePhysics.m_sphereRadius * pointToCentre.normalized;

            if (spherePhysics.m_mass != 0 && cubePhysics.m_mass != 0)
            {
                spherePhysics.m_velocity += netMovement * pointToCentre.normalized / 2;
                cubePhysics.m_velocity -= netMovement * pointToCentre.normalized / 2;
            }
            else if (spherePhysics.m_mass != 0)
                spherePhysics.m_velocity += netMovement * pointToCentre.normalized;
            else if (cubePhysics.m_mass != 0)
                cubePhysics.m_velocity += netMovement * pointToCentre.normalized;
        }
        //Else they're moving away from each other
    }

    /*void cubeOnPlane(Transform cubeTransform, Transform planeTransform, Physics cubePhysics, Physics planePhysics)
    {
        //Rotate vectors into worldspace
        Matrix4x4 cubeRotation = Matrix4x4.Rotate(Quaternion.Euler(cubeTransform.eulerAngles));

        Vector3 cubeX = cubeRotation.MultiplyVector(new Vector3(1, 0, 0)).normalized * cubeTransform.localScale.x * 0.5f;
        Vector3 cubeY = cubeRotation.MultiplyVector(new Vector3(0, 1, 0)).normalized * cubeTransform.localScale.y * 0.5f;
        Vector3 cubeZ = cubeRotation.MultiplyVector(new Vector3(0, 0, 1)).normalized * cubeTransform.localScale.z * 0.5f;

        Plane[] cubePlanes = new Plane[6];
        cubePlanes[0] = new Plane(planeTransform.position + cubeX, cubeX.normalized, cubeY, cubeZ);
        cubePlanes[1] = new Plane(planeTransform.position - cubeX, -cubeX.normalized, cubeY, cubeZ);
        cubePlanes[2] = new Plane(planeTransform.position + cubeY, cubeY.normalized, cubeX, cubeZ);
        cubePlanes[3] = new Plane(planeTransform.position - cubeY, -cubeY.normalized, cubeX, cubeZ);
        cubePlanes[4] = new Plane(planeTransform.position + cubeZ, cubeZ.normalized, cubeY, cubeX);
        cubePlanes[5] = new Plane(planeTransform.position - cubeZ, -cubeZ.normalized, cubeY, cubeX);

        Matrix4x4 planeRotation = Matrix4x4.Rotate(Quaternion.Euler(planeTransform.eulerAngles));

        Vector3 planeNormal = planeRotation.MultiplyVector(new Vector3(0, 1, 0)).normalized;
        Vector3 planeX = planeRotation.MultiplyVector(new Vector3(1, 0, 0)).normalized * planeTransform.localScale.x * 5;
        Vector3 planeZ = planeRotation.MultiplyVector(new Vector3(0, 0, 1)).normalized * planeTransform.localScale.z * 5;

        Plane plane = new Plane(planeTransform.position, planeNormal, planeX, planeZ);

        //Check for collision of any cube edges against the plane
        CollisionPoint finalCollision = new CollisionPoint(false);
        int count = 0;
        for (int i = 0; i < 6; i++)
        {
            CollisionPoint tempCollision = findCPPlaneOnPlane(plane, cubePlanes[i]);
            if (tempCollision.occurred)
            {
                count++;
                finalCollision.occurred = true;
                finalCollision.position += tempCollision.position;
            }
        }
        if (finalCollision.occurred)
        {
            finalCollision.position /= count;
            finalCollision.normal = planeNormal.normalized;

            float vCube = Vector3.Dot(cubePhysics.m_velocity, finalCollision.normal);
            float vPlane = Vector3.Dot(planePhysics.m_velocity, finalCollision.normal);
            float netMovement = vCube - vPlane;

            if (netMovement > m_contactSpeed) //Moving towards so collision should occur
            {
                float speedCube = speedAfterCollision(planePhysics.m_restitution * cubePhysics.m_restitution, vCube, vPlane, cubePhysics.m_mass, planePhysics.m_mass);
                cubePhysics.m_velocity += (speedCube - vCube) * finalCollision.normal;

                float speedPlane = speedAfterCollision(planePhysics.m_restitution * cubePhysics.m_restitution, vPlane, vCube, planePhysics.m_mass, cubePhysics.m_mass);
                planePhysics.m_velocity += (speedPlane - vPlane) * finalCollision.normal;
            }
            else if (netMovement > -m_contactSpeed) //In contact
            {
                sphereTransform.position = collision.position + spherePhysics.m_sphereRadius * pointToCentre.normalized;

                if (spherePhysics.m_mass != 0 && cubePhysics.m_mass != 0)
                {
                    spherePhysics.m_velocity += netMovement * pointToCentre.normalized / 2;
                    cubePhysics.m_velocity -= netMovement * pointToCentre.normalized / 2;
                }
                else if (spherePhysics.m_mass != 0)
                    spherePhysics.m_velocity += netMovement * pointToCentre.normalized;
                else if (cubePhysics.m_mass != 0)
                    cubePhysics.m_velocity += netMovement * pointToCentre.normalized;
            }
            //Else they're moving away from each other
        }
    }*/

    void SphereOnX(Transform firstTransform, Transform secondTransform, Physics secondPhysics)
    {
        switch (secondPhysics.m_collisionType)
        {
            case CollisionType.sphere:
                sphereOnSphere(firstTransform, secondTransform, this, secondPhysics);
                break;
            case CollisionType.plane:
                sphereOnPlane(firstTransform, secondTransform, this, secondPhysics);
                break;
            case CollisionType.cuboid:
                sphereOnCube(firstTransform, secondTransform, this, secondPhysics);
                break;
            default:
                break;
        }
    }

    void PlaneOnX(Transform firstTransform, Transform secondTransform, Physics secondPhysics)
    {
        switch (secondPhysics.m_collisionType)
        {
            case CollisionType.sphere:
                sphereOnPlane(secondTransform, firstTransform, secondPhysics, this);
                break;
            default:
                break;
        }
    }

    void CubeOnX(Transform firstTransform, Transform secondTransform, Physics secondPhysics)
    {
        switch (secondPhysics.m_collisionType)
        {
            case CollisionType.sphere:
                sphereOnCube(secondTransform, firstTransform, secondPhysics, this);
                break;
            default:
                break;
        }
    }

    void CollisionCheck(GameObject other)
    {
        Transform selfTransform = GetComponent<Transform>();
        Transform otherTransform = other.GetComponent<Transform>();
        Physics otherPhysics = other.GetComponent<Physics>();

        //Sphere collision check
        Vector3 centreToCentre = otherTransform.position - selfTransform.position;
        if ((m_sphereRadius + otherPhysics.m_sphereRadius) * (m_sphereRadius + otherPhysics.m_sphereRadius) < centreToCentre.sqrMagnitude)
            return;

        switch (m_collisionType)
        {
            case CollisionType.sphere:
                SphereOnX(selfTransform, otherTransform, otherPhysics);
                break;
            case CollisionType.plane:
                PlaneOnX(selfTransform, otherTransform, otherPhysics);
                break;
            case CollisionType.cuboid:
                CubeOnX(selfTransform, otherTransform, otherPhysics);
                break;
            default:
                break;
        }
    }

    // Update is called once per frame
    void Update()
    {
        Transform selfTransform = GetComponent<Transform>();
        Vector3 oldVelocity = m_velocity;

        //Gravity controls "In game""
        m_uniformAcceleration.x = m_gravityStrength * Input.GetAxis("Horizontal");
        m_uniformAcceleration.y = m_gravityStrength * Input.GetAxis("Vertical");
        if (Input.GetAxis("Horizontal") + Input.GetAxis("Vertical") == 0)
            m_uniformAcceleration.y = -m_gravityStrength;

        m_velocity += m_uniformAcceleration * Time.deltaTime;


        GameObject[] pulls = GameObject.FindGameObjectsWithTag("PhysicsObject");
        foreach (GameObject obj in pulls)
        {
            CollisionCheck(obj);
        }

        //Set transforms
        selfTransform.Translate((0.5f * m_velocity + 0.5f * oldVelocity) * Time.deltaTime);
    }
}