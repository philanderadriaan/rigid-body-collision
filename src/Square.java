import java.awt.Color;

import javax.media.j3d.Appearance;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.ColoringAttributes;
import javax.media.j3d.GeometryArray;
import javax.media.j3d.Node;
import javax.media.j3d.PointArray;
import javax.media.j3d.PointAttributes;
import javax.media.j3d.PolygonAttributes;
import javax.media.j3d.QuadArray;
import javax.media.j3d.Shape3D;
import javax.media.j3d.TriangleArray;
import javax.vecmath.Color3f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple2f;
import javax.vecmath.Vector2f;

/**
 * 
 * @author Daniel Beraun
 *
 */
public class Square extends Triangle
{
  private static final float[] VERTICES = {0, 0, 1, 0, 1, 1, 0, 1};

  private float width;
  private Vector2f[] vertexCache;
  private Vector2f[] normalCache;

  public Square(float mass, float positionX, float positionY, float velocityX,
                  float velocityY, float orientation, float angularVelocity, float width,
                  Color3f color)
  {
    super( mass, positionX, positionY,
          velocityX, velocityY, orientation,
          angularVelocity, width, color, false);

    if (width <= 0)
      throw new IllegalArgumentException();

    // True center of mass for a square
    centerOfMass.x = centerOfMass.y = width / 2;
    momentOfInertia = (float) (Math.pow(width, 3) / 12);
    this.width = width;
    TG.addChild(createShape(width, color));
  }

  public Square(float mass, Tuple2f position, Tuple2f velocity, float orientation,
                  float angularVelocity, float width, Color3f color)
  {
    this(mass, position.x, position.y, velocity.x, velocity.y, orientation, angularVelocity,
         width, color);
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
        float tmpX = VERTICES[i] * width;
        float tmpY = VERTICES[i + 1] * width;
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
      Vector2f[] vertices = getVertices();
      normalCache = new Vector2f[vertices.length];

      for (int i = 0; i < vertices.length; i++)
      {
        normalCache[i] = new Vector2f();
        normalCache[i].scaleAdd(-1, vertices[i], vertices[(i + 1) % vertices.length]);
        normalCache[i].normalize();
        float tmp = normalCache[i].x;
        normalCache[i].x = normalCache[i].y;
        normalCache[i].y = -tmp;
      }
    }
    return normalCache;
  }

  private Node createShape(float width, Color3f color)
  {
    QuadArray geometry = new QuadArray(4, GeometryArray.COORDINATES);
    for (int i = 0; i < VERTICES.length; i += 2)
      geometry.setCoordinate(i / 2, new Point3f(width * VERTICES[i], width * VERTICES[i + 1],
                                                0));

    PointArray centerOfMassGeometry = new PointArray(1, GeometryArray.COORDINATES);
    centerOfMassGeometry.setCoordinate(0, new Point3f(centerOfMass.x, centerOfMass.y, 0));

    BranchGroup root = new BranchGroup();
    if (color == null)
      color = new Color3f(Color.getHSBColor((float) Math.random(), 1, 1));
    Appearance appearance = new Appearance();
    appearance
        .setColoringAttributes(new ColoringAttributes(color, ColoringAttributes.FASTEST));
    PolygonAttributes polyAttr =
        new PolygonAttributes(PolygonAttributes.POLYGON_FILL, PolygonAttributes.CULL_NONE, 0);
    appearance.setPolygonAttributes(polyAttr);
    root.addChild(new Shape3D(geometry, appearance));

    appearance = new Appearance();
    appearance.setPointAttributes(new PointAttributes(4, true));
    root.addChild(new Shape3D(centerOfMassGeometry, appearance));

    return root;
  }
}
