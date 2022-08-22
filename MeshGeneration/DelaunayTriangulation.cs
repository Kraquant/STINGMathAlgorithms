namespace MeshGeneration
{
    /// <summary>
    /// This Class Creates a Delaunay Triangulation for a set of points
    /// A Super Triangle must be given to define an outside boundary
    /// A constructor with no Super Triangle is also available. The super triangle will be created procedurally
    /// </summary>
    public class DelaunayTriangulation
    {
        private List<Vector2> _points;
        private List<Vector3Int> _triangles;
        private bool _superTriangleRemoved;

        public Vector2[] Points { get { return _points.ToArray(); } }
        public Vector3Int[] Triangles { get { return _triangles.ToArray(); } }

        #region Constructors
        private DelaunayTriangulation()
        {
            _points = new List<Vector2>();
            _triangles = new List<Vector3Int>();
            _superTriangleRemoved = false;
        }
        public DelaunayTriangulation(List<Vector2> points, Vector2[] superTriangle) : this()
        {
            if (superTriangle.Length != 3) throw new System.Exception("Super Triangle Undefined");

            _points.AddRange(superTriangle);
            _points.AddRange(points);

            _triangles.Add(new Vector3Int(0, 1, 2));

            //For Everypoint
            if (_points.Count > 3)
            {
                for (int i = 3; i < _points.Count; i++)
                {
                    AddPoint(i);
                }
            }

        }
        public DelaunayTriangulation(List<Vector2> points) : this(points, ConstructSuperTriangle(points.ToArray()))
        { }
        #endregion

        #region public methods
        static public void CircumCircle(Vector2 A, Vector2 B, Vector2 C, out Vector2 O, out float R)
        {
            Vector4 idCol = new Vector4(0, 0, 0, 1);
            Matrix4x4 deltaMatrix = new Matrix4x4(
                new Vector4(A.x, B.x, C.x, 0),
                new Vector4(A.y, B.y, C.y, 0),
                new Vector4(1, 1, 1, 0),
                idCol);

            float delta = 2 * deltaMatrix.determinant;

            Vector4 col1 = new Vector4(
                A.x * A.x + A.y * A.y,
                B.x * B.x + B.y * B.y,
                C.x * C.x + C.y * C.y,
                0);
            Vector3 col3 = new Vector4(
                1, 1, 1, 0);


            Matrix4x4 OXMat = new Matrix4x4(
                col1,
                new Vector4(A.y, B.y, C.y, 0),
                col3,
                idCol);

            Matrix4x4 OYMat = new Matrix4x4(
                col1,
                new Vector4(A.x, B.x, C.x, 0),
                col3,
                idCol);

            O = new Vector2(OXMat.determinant / delta, -OYMat.determinant / delta);

            //Radius

            float a = (C - B).magnitude;
            float b = (C - A).magnitude;
            float c = (B - A).magnitude;

            float p = (a + b + c) / 2;

            R = (a * b * c) / (4 * Mathf.Sqrt(p * (p - a) * (p - b) * (p - c)));


        }
        static public Vector2[] ConstructSuperTriangle(Vector2[] points)
        {
            float minX = float.PositiveInfinity;
            float maxX = float.NegativeInfinity;
            float minY = float.PositiveInfinity;
            float maxY = float.NegativeInfinity;

            foreach (Vector2 point in points)
            {
                if (point.x < minX) minX = point.x;
                if (point.x > maxX) maxX = point.x;
                if (point.y < minY) minY = point.y;
                if (point.y > maxY) maxY = point.y;
            }

            Vector2 center = new Vector2((maxX + minX) / 2, (maxY + minY) / 2);

            float radius = Mathf.Sqrt(((maxX - minX) / 2) * ((maxX - minX) / 2) + ((maxY - minY) / 2) * ((maxY - minY) / 2));

            return GetCircleOutsideTriangle(center, radius);

        }
        static public Vector2[] GetCircleOutsideTriangle(Vector2 center, float radius)
        {
            Vector2[] points = new Vector2[3];

            points[0] = center + new Vector2(1, 0) * 2 * radius;
            points[1] = center + new Vector2(Mathf.Cos(120 * Mathf.Deg2Rad), Mathf.Sin(120 * Mathf.Deg2Rad)) * 2 * radius;
            points[2] = center + new Vector2(Mathf.Cos(240 * Mathf.Deg2Rad), Mathf.Sin(240 * Mathf.Deg2Rad)) * 2 * radius;

            return points;
        }
        static public Vector3Int[] RemoveOutsideTriangles(Vector2Int[] borderEdges, Vector3Int[] triangles)
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

            return res.ToArray();
        }
        public void OrientTriangles()
        {
            for (int i = 0; i < _triangles.Count; i++)
            {
                Vector2 XY = _points[_triangles[i].y] - _points[_triangles[i].x];
                Vector2 XZ = _points[_triangles[i].z] - _points[_triangles[i].x];

                if (Vector3.Dot(Vector3.Cross(XY, XZ), Vector3.forward) > 0)
                {
                    Vector3Int swap = new Vector3Int(_triangles[i].x, _triangles[i].z, _triangles[i].y);
                    _triangles[i] = swap;
                }
            }
        }
        public void RemoveSuperTriangle()
        {
            if (_superTriangleRemoved) return;
            _superTriangleRemoved = true;
            List<Vector3Int> goodTriangles = new List<Vector3Int>();

            foreach (Vector3Int triangle in _triangles)
            {
                if (triangle.x == 0 || triangle.x == 1 || triangle.x == 2 ||
                    triangle.y == 0 || triangle.y == 1 || triangle.y == 2 ||
                    triangle.z == 0 || triangle.z == 1 || triangle.z == 2) continue;
                goodTriangles.Add(new Vector3Int(triangle.x - 3, triangle.y - 3, triangle.z - 3));
            }
            _triangles = goodTriangles;

            _points.RemoveRange(0, 3);

        }
        #endregion

        #region private Methods

        private void AddPoint(int index)
        {
            List<Vector3Int> goodTriangles = new List<Vector3Int>();
            List<Vector3Int> badTriangles = new List<Vector3Int>();

            for (int i = 0; i < _triangles.Count; i++)
            {
                if (IsInCircumCircle(_triangles[i], index))
                {
                    badTriangles.Add(_triangles[i]);
                }
                else
                {
                    goodTriangles.Add(_triangles[i]);
                }
            }

            //Spliting triangles into edges
            List<Vector2Int> goodEdges = new List<Vector2Int>();
            List<Vector2Int> badEdges = new List<Vector2Int>();

            foreach (Vector3Int triangle in badTriangles)
            {
                badEdges.Add(new Vector2Int(triangle.x, triangle.y));
                badEdges.Add(new Vector2Int(triangle.x, triangle.z));
                badEdges.Add(new Vector2Int(triangle.z, triangle.y));
            }

            for (int i = 0; i < badEdges.Count; i++)
            {
                bool isGood = true;
                int j = 0;
                while (j < goodEdges.Count)
                {
                    if (SameEdge(goodEdges[j], badEdges[i]))
                    {
                        isGood = false;
                        break;
                    }
                    j++;
                }

                if (isGood) goodEdges.Add(badEdges[i]);
                else
                {
                    goodEdges.RemoveAt(j);
                }
            }

            //Now we create triangles with the good remaining edges
            //And we add them to the good triangles

            foreach (Vector2Int edge in goodEdges)
            {
                goodTriangles.Add(new Vector3Int(edge.x, edge.y, index));
            }

            //Finally we update the variable containing all the triangles
            _triangles = goodTriangles;

        }
        private bool SameEdge(Vector2Int E1, Vector2Int E2)
        {
            return (E1.x == E2.x && E1.y == E2.y) || (E1.x == E2.y && E1.y == E2.x);
        }
        private bool IsInCircumCircle(Vector3Int triangle, Vector2 point)
        {
            CircumCircle(_points[triangle.x], _points[triangle.y], _points[triangle.z], out Vector2 circumO, out float circumR);
            return Vector2.Distance(point, circumO) <= circumR;
        }
        private bool IsInCircumCircle(Vector3Int triangle, int pointIndex)
        {
            return IsInCircumCircle(triangle, _points[pointIndex]);
        }

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
        static private void SharedEdges(List<Vector3Int> triangles, out List<Vector3Int> insideTriangles, out List<SharedTriangle> borderTriangles)
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
        #endregion
    }
}