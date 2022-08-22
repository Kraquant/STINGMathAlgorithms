private class SharedTriangle
{
    public Vector3Int triangle;
    public bool shareXY;
    public bool shareXZ;
    public bool shareYZ;

    public SharedTriangle(Vector3Int initTriangle, bool initShareXY, bool initShareXZ, bool initShareYZ)
    {
        triangle = initTriangle;
        shareXY = initShareXY;
        shareXZ = initShareXZ;
        shareYZ = initShareYZ;
    }
}

/// <summary>
/// Method used to create a mesh with just a set of points
/// </summary>
/// <param name="points"></param>
/// <param name="OriginWorldPos"></param>
/// <param name="originToGeometry"></param>
/// <returns></returns>
private Mesh CreateRoadMesh(List<Vector3> points, float alpha, out Vector3 OriginWorldPos, bool originToGeometry = false)
{
    List<Vector2> points2d = new List<Vector2>();
    foreach (Vector3 point in points) points2d.Add(new Vector2(point.x, point.z));

    //Remove inside points using the alpha shape algorithm
    AlphaShape shape = new AlphaShape(points2d.ToArray(), alpha);
    points2d.Clear();
    HashSet<int> outsidePoints = new HashSet<int>();
    for (int i = 0; i < shape.Segments.Length; i++)
    {
        outsidePoints.Add(shape.Segments[i].x);
        outsidePoints.Add(shape.Segments[i].y);
    }
    foreach (int pointIndex in outsidePoints) points2d.Add(shape.Points[pointIndex]);
    //Recreating the alpha shape using border points to reconnect them
    shape = new AlphaShape(points2d.ToArray(), alpha);


    //Creating the final mesh
    List<Vector3> vertices = new List<Vector3>();
    List<Vector3> normals = new List<Vector3>();
    List<int> tris = new List<int>();
    List<Vector2> textureCoordinates = new List<Vector2>();

    //Triangulation
    DelaunayTriangulation triangulation = new DelaunayTriangulation(shape.Points.ToList());
    triangulation.RemoveSuperTriangle();
    triangulation.OrientTriangles();
    //Creating the vertice list
    foreach (Vector2 point in shape.Points) vertices.Add(new Vector3(point.x, 0, point.y));

    OriginWorldPos = Vector3.zero;
    if (originToGeometry)
    {
        for (int i = 0; i < vertices.Count; i++) OriginWorldPos += vertices[i];
        OriginWorldPos /= vertices.Count;
        for (int i = 0; i < vertices.Count; i++) vertices[i] = vertices[i] - OriginWorldPos;
    }

    //Creating the normals and temporary uvs
    for (int i = 0; i < vertices.Count; i++)
    {
        normals.Add(Vector3.up);
        textureCoordinates.Add(new Vector2(vertices[i].x, vertices[i].z));
    }

    //Creating tris
    List<Vector2Int> border = shape.Segments.ToList();
    List<Vector3Int> insideTris = RemoveOutsideTriangles(border.ToArray(), triangulation.Triangles); //TODO: Kind of work
    foreach (Vector3Int triangle in insideTris)
    {
        tris.Add(triangle.x);
        tris.Add(triangle.y);
        tris.Add(triangle.z);
    }

    //Making the final mesh
    Mesh laneMesh = new Mesh();
    laneMesh.vertices = vertices.ToArray();
    laneMesh.triangles = tris.ToArray();
    laneMesh.normals = normals.ToArray();
    laneMesh.uv = textureCoordinates.ToArray();

    /*borderIndex = shape.Segments.ToList();
    points3D = vertices;
    outsideTriangle = DelaunayTriangulation.ConstructSuperTriangle(shape.Points);*/

    return laneMesh;

}

private List<Vector3Int> RemoveOutsideTriangles(Vector2Int[] borderEdges, Vector3Int[] triangles)
{
    List<Vector3Int> res = new List<Vector3Int>();
    res.AddRange(triangles);
    HashSet<int> borderPoints = new HashSet<int>();
    foreach (Vector2Int borderEdge in borderEdges)
    {
        borderPoints.Add(borderEdge.x);
        borderPoints.Add(borderEdge.y);
    }

    bool onlyGoodTris = false;
    while (!onlyGoodTris)
    {
        SharedEdges(res, out List<Vector3Int> insideTris, out List<SharedTriangle> borderTris);

        onlyGoodTris = true;
        foreach (SharedTriangle borderTri in borderTris)
        {
            bool isInside = true;

            if (!borderTri.shareXY)
            {
                isInside = false;
                foreach (Vector2Int borderEdge in borderEdges)
                {
                    if ((borderTri.triangle.x == borderEdge.x && borderTri.triangle.y == borderEdge.y) ||
                        (borderTri.triangle.x == borderEdge.y && borderTri.triangle.y == borderEdge.x))
                    {
                        isInside = true;
                        break;
                    }
                }
            }
            if (!borderTri.shareXZ)
            {
                isInside = false;
                foreach (Vector2Int borderEdge in borderEdges)
                {
                    if ((borderTri.triangle.x == borderEdge.x && borderTri.triangle.z == borderEdge.y) ||
                        (borderTri.triangle.x == borderEdge.y && borderTri.triangle.z == borderEdge.x))
                    {
                        isInside = true;
                        break;
                    }
                }
            }
            if (!borderTri.shareYZ)
            {
                isInside = false;
                foreach (Vector2Int borderEdge in borderEdges)
                {
                    if ((borderTri.triangle.y == borderEdge.x && borderTri.triangle.z == borderEdge.y) ||
                        (borderTri.triangle.y == borderEdge.y && borderTri.triangle.z == borderEdge.x))
                    {
                        isInside = true;
                        break;
                    }
                }
            }

            if (isInside)
            {
                insideTris.Add(borderTri.triangle);
            }
            else
            {
                onlyGoodTris = false;
            }
        }
        res = insideTris;

    }

    return res;
}

private void SharedEdges(List<Vector3Int> triangles, out List<Vector3Int> insideTriangles, out List<SharedTriangle> borderTriangles)
{
    insideTriangles = new List<Vector3Int>();
    borderTriangles = new List<SharedTriangle>();

    for (int i = 0; i < triangles.Count; i++)
    {
        bool shareXY = false;
        bool shareXZ = false;
        bool shareYZ = false;
        for (int j = 0; j < triangles.Count; j++)
        {
            if (i == j) continue;
            if ((triangles[i].x == triangles[j].x && triangles[i].y == triangles[j].y) ||
                (triangles[i].x == triangles[j].y && triangles[i].y == triangles[j].x) ||

                (triangles[i].x == triangles[j].z && triangles[i].y == triangles[j].y) ||
                (triangles[i].x == triangles[j].y && triangles[i].y == triangles[j].z) ||

                (triangles[i].x == triangles[j].x && triangles[i].y == triangles[j].z) ||
                (triangles[i].x == triangles[j].z && triangles[i].y == triangles[j].x)) shareXY = true;

            if ((triangles[i].x == triangles[j].x && triangles[i].z == triangles[j].y) ||
                (triangles[i].x == triangles[j].y && triangles[i].z == triangles[j].x) ||

                (triangles[i].x == triangles[j].z && triangles[i].z == triangles[j].y) ||
                (triangles[i].x == triangles[j].y && triangles[i].z == triangles[j].z) ||

                (triangles[i].x == triangles[j].x && triangles[i].z == triangles[j].z) ||
                (triangles[i].x == triangles[j].z && triangles[i].z == triangles[j].x)) shareXZ = true;

            if ((triangles[i].y == triangles[j].x && triangles[i].z == triangles[j].y) ||
                (triangles[i].y == triangles[j].y && triangles[i].z == triangles[j].x) ||

                (triangles[i].y == triangles[j].z && triangles[i].z == triangles[j].y) ||
                (triangles[i].y == triangles[j].y && triangles[i].z == triangles[j].z) ||

                (triangles[i].y == triangles[j].x && triangles[i].z == triangles[j].z) ||
                (triangles[i].y == triangles[j].z && triangles[i].z == triangles[j].x)) shareYZ = true;

            if (shareXY && shareXZ && shareYZ) break;


        }

        if (shareXY && shareXZ && shareYZ) insideTriangles.Add(triangles[i]);
        else borderTriangles.Add(new SharedTriangle(triangles[i], shareXY, shareXZ, shareYZ));
    }
}

