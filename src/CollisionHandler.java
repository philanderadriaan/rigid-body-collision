import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

/**
 * 
 * @author Daniel Beraun
 * @author Phil Adriaan
 *
 */
public class CollisionHandler
{
  private static final float COEFFICIENT_OF_RESTITUTION = 0.9f;

  public static void checkAndResolveCollision(final PhysicsObject a, final PhysicsObject b)
  {
    final CollisionInfo ci = getCollisionInfo(a, b);
    if (ci == null)
    {
      return;
    }

    // Vector from the center of mass of object a to the collision point
    final Vector2f r_ap = new Vector2f();
    r_ap.scaleAdd(-1, a.getGlobalCenterOfMass(), ci.position);
    // Vector from the center of mass of object b to the collision point
    final Vector2f r_bp = new Vector2f();
    r_bp.scaleAdd(-1, b.getGlobalCenterOfMass(), ci.position);
    // Velocity of object a at the point of collision
    final Vector2f v_ap1 = new Vector2f();
    v_ap1.x = a.velocity.x - a.angularVelocity * r_ap.y;
    v_ap1.y = a.velocity.y + a.angularVelocity * r_ap.x;
    // Velocity of object b at the point of collision
    final Vector2f v_bp1 = new Vector2f();
    v_bp1.x = b.velocity.x - b.angularVelocity * r_bp.y;
    v_bp1.y = b.velocity.y + b.angularVelocity * r_bp.x;
    // The collision impulse
    final Vector2f v_ab1 = new Vector2f();
    v_ab1.scaleAdd(-1, v_bp1, v_ap1);
    final float tmpA = r_ap.x * ci.normal.y - r_ap.y * ci.normal.x;
    final float tmpB = r_bp.x * ci.normal.y - r_bp.y * ci.normal.x;
    final float j =
        -(1 + COEFFICIENT_OF_RESTITUTION) *
            v_ab1.dot(ci.normal) /
            (1 / a.mass + 1 / b.mass + tmpA * tmpA / a.momentOfInertia + tmpB * tmpB /
                                                                         b.momentOfInertia);
    // Update object a's velocity
    a.velocity.scaleAdd(j / a.mass, ci.normal, a.velocity);
    // Update object b's velocity
    b.velocity.scaleAdd(-j / b.mass, ci.normal, b.velocity);
    // Update object a's angular velocity
    a.angularVelocity += j * (r_ap.x * ci.normal.y - r_ap.y * ci.normal.x) / a.momentOfInertia;
    // Update object b's angular velocity
    b.angularVelocity -= j * (r_bp.x * ci.normal.y - r_bp.y * ci.normal.x) / b.momentOfInertia;
    // Remove object overlap

    a.position.scaleAdd(-ci.depth / (a.mass * (1 / a.mass + 1 / b.mass)), ci.normal,
                        a.position);
    b.position
        .scaleAdd(ci.depth / (b.mass * (1 / a.mass + 1 / b.mass)), ci.normal, b.position);

    // Overlap Resolution using rotation
    Vector2f l = new Vector2f();
    l.scale(1, ci.normal);
    l.normalize();
    l.scale(ci.depth, l);
    Vector2f la = new Vector2f();
    la.scaleAdd(1, ci.position, l);
    l.scale(2, l);
    Vector2f lb = new Vector2f();
    lb.scaleAdd(1, ci.position, l);
    Vector2f a_a = new Vector2f(la.x - b.position.x, la.y - b.position.y);
    Vector2f a_b = new Vector2f(lb.x - b.position.x, lb.y - b.position.y);
    float aa = a_a.length();
    float ab = a_b.length();
    float ac = ci.depth;
    float angle =
        (float) Math.acos((Math.pow(aa, 2) + Math.pow(ab, 2) - Math.pow(ac, 2)) / 2 * aa * ab);
    if (!Float.isNaN(angle))
    {
      angle = (float) Math.toRadians(angle);
      // System.out.println(angle);
      Vector3f cross = new Vector3f();
      Vector3f la3 = new Vector3f(la.x, la.y, 0);
      Vector3f lb3 = new Vector3f(lb.x, lb.y, 0);
      cross.cross(la3, lb3);
      if (cross.z > 0)
      {
        b.orientation += angle / b.mass;
      }
      else
      {
        b.orientation -= angle / b.mass;
      }
    }
    // End of overlap resolution
    
    a.clearCaches();
    b.clearCaches();
  }

  private static CollisionInfo getCollisionInfo(final PhysicsObject a, final PhysicsObject b)
  {
    if (a == b)
    {
      return null;
    }
    CollisionInfo ci = null;
    if (a instanceof HalfSpace)
    {
      if (b instanceof Circle)
      {
        ci = getCollision((HalfSpace) a, (Circle) b);
      }
      else if (b instanceof Triangle)
      {
        ci = getCollision((HalfSpace) a, (Triangle) b);
      }
    }
    else if (a instanceof Circle)
    {
      if (b instanceof Circle)
      {
        ci = getCollision((Circle) a, (Circle) b);
      }
      else if (b instanceof Triangle)
      {
        ci = getCollision((Circle) a, (Triangle) b);
      }
    }
    else if (a instanceof Triangle)
    {
      if (b instanceof Triangle)
      {
        ci = getCollision((Triangle) a, (Triangle) b);
      }
      else if (b instanceof Circle)
      {
        ci = getCollision((Circle) b, (Triangle) a);
      }
    }
    return ci;
  }

  private static CollisionInfo getCollision(final Circle a, final Triangle b)
  {
    final Vector2f[] verticesB = b.getVertices();
    final Vector2f[] normalsB = b.getNormals();
    final float[][] distanceFromA = new float[1][verticesB.length];
    final float[][] distanceFromB = new float[verticesB.length][1];
    final int[] indexMinDistanceFromA = new int[1];
    final int[] indexMinDistanceFromB = new int[verticesB.length];
    for (int i = 0; i < 1; i++)
    {
      for (int j = 0; j < verticesB.length; j++)
      {
        final Vector2f tmp = new Vector2f();
        tmp.scaleAdd(-1, a.position, verticesB[j]);
        Vector2f normal = new Vector2f();
        normal.negate(normalsB[j]);
        distanceFromA[i][j] = tmp.dot(normal) - a.radius;
        if (distanceFromA[i][j] < distanceFromA[i][indexMinDistanceFromA[i]])
        {
          indexMinDistanceFromA[i] = j;
        }
      }
      if (distanceFromA[i][indexMinDistanceFromA[i]] >= 0)
      {
        return null;
      }
    }
    for (int i = 0; i < verticesB.length; i++)
    {
      for (int j = 0; j < 1; j++)
      {
        final Vector2f tmp = new Vector2f(a.position);
        tmp.scaleAdd(-1, verticesB[i], a.position);
        distanceFromB[i][j] = tmp.dot(normalsB[i]) - a.radius;
        if (distanceFromB[i][j] < distanceFromB[i][indexMinDistanceFromB[i]])
        {
          indexMinDistanceFromB[i] = j;
        }
      }
      if (distanceFromB[i][indexMinDistanceFromB[i]] >= 0)
      {
        return null;
      }
    }
    int indexMaxDistanceFromA = 0;
    for (int i = 1; i < 1; i++)
    {
      if (distanceFromA[i][indexMinDistanceFromA[i]] > distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]])
      {
        indexMaxDistanceFromA = i;
      }
    }
    int indexMaxDistanceFromB = 0;
    for (int i = 1; i < verticesB.length; i++)
    {
      if (distanceFromB[i][indexMinDistanceFromB[i]] > distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]])
      {
        indexMaxDistanceFromB = i;
      }
    }
    final CollisionInfo ci = new CollisionInfo();
    if (distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]] > distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]])
    {
      ci.depth =
          -distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]];
      Vector2f normal = new Vector2f();
      normal.negate(normalsB[indexMaxDistanceFromA]);
      ci.normal = new Vector2f(normal);
      ci.position = new Vector2f(verticesB[indexMinDistanceFromA[indexMaxDistanceFromA]]);
      ci.position.scaleAdd(-ci.depth, ci.normal, ci.position);
    }
    else
    {
      ci.depth =
          -distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]];
      ci.normal = new Vector2f(normalsB[indexMaxDistanceFromB]);
      ci.normal.scale(-1);
      ci.position = new Vector2f(a.position);
    }
    return ci;
  }

  private static CollisionInfo getCollision(final HalfSpace a, final Circle b)
  {
    final float distance = a.normal.dot(b.position) - a.intercept - b.radius;
    if (distance < 0)
    {
      final CollisionInfo ci = new CollisionInfo();
      ci.normal = a.normal;
      ci.depth = -distance;
      ci.position = new Vector2f();
      ci.position.scaleAdd(-(b.radius - ci.depth), ci.normal, b.position);
      return ci;
    }
    return null;
  }

  private static CollisionInfo getCollision(final HalfSpace a, final Triangle b)
  {
    final Vector2f[] vertices = b.getVertices();
    final float[] distances = new float[vertices.length];

    for (int i = 0; i < vertices.length; i++)
    {
      distances[i] = a.normal.dot(vertices[i]) - a.intercept;
    }

    int minIndex = 0;
    for (int i = 1; i < distances.length; i++)
    {
      if (distances[i] < distances[minIndex])
      {
        minIndex = i;
      }
    }
    if (distances[minIndex] >= 0)
    {
      return null;
    }

    final CollisionInfo ci = new CollisionInfo();
    ci.depth = -distances[minIndex];
    ci.normal = a.normal;
    ci.position = new Vector2f(vertices[minIndex]);
    ci.position.scaleAdd(ci.depth, ci.normal, ci.position);
    return ci;
  }

  private static CollisionInfo getCollision(final Circle a, final Circle b)
  {
    final Vector2f n = new Vector2f();
    n.scaleAdd(-1, a.position, b.position);
    float distance = n.length() - a.radius - b.radius;
    if (distance < 0)
    {
      final CollisionInfo ci = new CollisionInfo();
      n.normalize();
      ci.normal = n;
      if (distance < 0)
      {
        ci.depth = -distance;
      }
      else
      {
        ci.depth = 0;
      }
      ci.position = new Vector2f();
      ci.position.scaleAdd(a.radius - ci.depth / 2, ci.normal, a.position);
      return ci;
    }
    return null;
  }

  private static CollisionInfo getCollision(final Triangle a, final Triangle b)
  {
    final Vector2f[] verticesA = a.getVertices();
    final Vector2f[] normalsA = a.getNormals();
    final Vector2f[] verticesB = b.getVertices();
    final Vector2f[] normalsB = b.getNormals();
    final float[][] distanceFromA = new float[verticesA.length][verticesB.length];
    final float[][] distanceFromB = new float[verticesB.length][verticesA.length];
    final int[] indexMinDistanceFromA = new int[verticesA.length];
    final int[] indexMinDistanceFromB = new int[verticesB.length];
    for (int i = 0; i < verticesA.length; i++)
    {
      for (int j = 0; j < verticesB.length; j++)
      {
        final Vector2f tmp = new Vector2f();
        tmp.scaleAdd(-1, verticesA[i], verticesB[j]);
        distanceFromA[i][j] = tmp.dot(normalsA[i]);
        if (distanceFromA[i][j] < distanceFromA[i][indexMinDistanceFromA[i]])
        {
          indexMinDistanceFromA[i] = j;
        }
      }
      if (distanceFromA[i][indexMinDistanceFromA[i]] >= 0)
      {
        return null;
      }
    }
    for (int i = 0; i < verticesB.length; i++)
    {
      for (int j = 0; j < verticesA.length; j++)
      {
        final Vector2f tmp = new Vector2f(verticesA[j]);
        tmp.scaleAdd(-1, verticesB[i], verticesA[j]);
        distanceFromB[i][j] = tmp.dot(normalsB[i]);
        if (distanceFromB[i][j] < distanceFromB[i][indexMinDistanceFromB[i]])
        {
          indexMinDistanceFromB[i] = j;
        }
      }
      if (distanceFromB[i][indexMinDistanceFromB[i]] >= 0)
      {
        return null;
      }
    }
    int indexMaxDistanceFromA = 0;
    for (int i = 1; i < verticesA.length; i++)
    {
      if (distanceFromA[i][indexMinDistanceFromA[i]] > distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]])
      {
        indexMaxDistanceFromA = i;
      }
    }
    int indexMaxDistanceFromB = 0;
    for (int i = 1; i < verticesB.length; i++)
    {
      if (distanceFromB[i][indexMinDistanceFromB[i]] > distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]])
      {
        indexMaxDistanceFromB = i;
      }
    }
    final CollisionInfo ci = new CollisionInfo();
    if (distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]] > distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]])
    {
      ci.depth =
          -distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]];
      ci.normal = new Vector2f(normalsA[indexMaxDistanceFromA]);
      ci.position = new Vector2f(verticesB[indexMinDistanceFromA[indexMaxDistanceFromA]]);
      ci.position.scaleAdd(-ci.depth, ci.normal, ci.position);
    }
    else
    {
      ci.depth =
          -distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]];
      ci.normal = new Vector2f(normalsB[indexMaxDistanceFromB]);
      ci.normal.scale(-1);
      ci.position = new Vector2f(verticesA[indexMinDistanceFromB[indexMaxDistanceFromB]]);
      // ci.position.scaleAdd(ci.depth, ci.normal, ci.position);
    }
    return ci;
  }
}
