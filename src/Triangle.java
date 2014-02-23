import java.awt.Color;

import javax.media.j3d.Appearance;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.ColoringAttributes;
import javax.media.j3d.GeometryArray;
import javax.media.j3d.Node;
import javax.media.j3d.PointArray;
import javax.media.j3d.PointAttributes;
import javax.media.j3d.PolygonAttributes;
import javax.media.j3d.Shape3D;
import javax.media.j3d.TriangleArray;
import javax.vecmath.Color3f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple2f;
import javax.vecmath.Vector2f;

// Isosceles right triangle
public class Triangle extends PhysicsObject
{
  private static final float[] VERTICES = {0, 0, 1, 0, 0, 1};

  private float width;
  private Vector2f[] vertexCache;
  private Vector2f[] normalCache;

  public Triangle(final float mass, final float positionX, final float positionY,
                  final float velocityX, final float velocityY, final float orientation,
                  final float angularVelocity, final float width, final Color3f color,
                  boolean triangle)
  {
    super(mass, positionX, positionY, velocityX, velocityY, orientation, angularVelocity);

    if (width <= 0)
    {
      throw new IllegalArgumentException();
    }

    // True center of mass for an isosceles right triangle
    centerOfMass.x = centerOfMass.y = width / 3;
    momentOfInertia = (float) (Math.pow(width, 4) / 18);
    this.width = width;
    if (triangle)
    {
      TG.addChild(createShape(width, color));
    }
  }

  public Triangle(final float mass, final Tuple2f position, final Tuple2f velocity,
                  final float orientation, final float angularVelocity, final float width,
                  final Color3f color, boolean triangle)
  {
    this(mass, position.x, position.y, velocity.x, velocity.y, orientation, angularVelocity,
         width, color, triangle);
  }

  public void clearCaches()
  {
    vertexCache = null;
    normalCache = null;
  }

  public Vector2f[] getVertices()
  {
    if (vertexCache == null)
    {
      vertexCache = new Vector2f[VERTICES.length / 2];
      for (int i = 0; i < VERTICES.length; i += 2)
      {
        final float tmpX = VERTICES[i] * width;
        final float tmpY = VERTICES[i + 1] * width;
        vertexCache[i / 2] = new Vector2f();
        vertexCache[i / 2].x =
            (float) (Math.cos(orientation) * tmpX - Math.sin(orientation) * tmpY) + position.x;
        vertexCache[i / 2].y =
            (float) (Math.sin(orientation) * tmpX + Math.cos(orientation) * tmpY) + position.y;
      }
    }
    return vertexCache;
  }

  public Vector2f[] getNormals()
  {
    if (normalCache == null)
    {
      final Vector2f[] vertices = getVertices();
      normalCache = new Vector2f[vertices.length];

      for (int i = 0; i < vertices.length; i++)
      {
        normalCache[i] = new Vector2f();
        normalCache[i].scaleAdd(-1, vertices[i], vertices[(i + 1) % vertices.length]);
        normalCache[i].normalize();
        final float tmp = normalCache[i].x;
        normalCache[i].x = normalCache[i].y;
        normalCache[i].y = -tmp;
      }
    }
    return normalCache;
  }

  private Node createShape(final float width, Color3f color)
  {
    final TriangleArray geometry = new TriangleArray(3, GeometryArray.COORDINATES);
    for (int i = 0; i < VERTICES.length; i += 2)
    {
      geometry.setCoordinate(i / 2, new Point3f(width * VERTICES[i], width * VERTICES[i + 1],
                                                0));
    }

    final PointArray centerOfMassGeometry = new PointArray(1, GeometryArray.COORDINATES);
    centerOfMassGeometry.setCoordinate(0, new Point3f(centerOfMass.x, centerOfMass.y, 0));

    final BranchGroup root = new BranchGroup();
    if (color == null)
    {
      color =
          new Color3f(Color.getHSBColor((float) Math.random(), (float) Math.random(),
                                        (float) Math.max(Math.random(), 0.5)));
    }
    Appearance appearance = new Appearance();
    appearance
        .setColoringAttributes(new ColoringAttributes(color, ColoringAttributes.FASTEST));
    final PolygonAttributes polyAttr =
        new PolygonAttributes(PolygonAttributes.POLYGON_FILL, PolygonAttributes.CULL_NONE, 0);
    appearance.setPolygonAttributes(polyAttr);
    root.addChild(new Shape3D(geometry, appearance));

    appearance = new Appearance();
    appearance.setPointAttributes(new PointAttributes(4, true));
    root.addChild(new Shape3D(centerOfMassGeometry, appearance));

    return root;
  }
}
