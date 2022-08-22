using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

    namespace MeshGeneration
    {
        /// <summary>
        /// Class Representing the alpha shape of a set of points
        /// This class is used to determine the contour of a set of 2D points
        /// </summary>
        public class AlphaShape //Works in 2D
        {
            static readonly float EPSILON = 0.00001f;
            private List<Vector2Int> _segments;
            private Vector2[] _points;
            private List<Vector2> _alphaCircles;

            private float _alpha;

            public Vector2Int[] Segments { get { return _segments.ToArray(); } }
            public float Alpha { get { return _alpha; } }
            public Vector2[] Points { get { return _points; } }
            public Vector2[] AlphaCircles { get { return _alphaCircles.ToArray(); } }

            public AlphaShape(Vector2[] points, float alpha)
            {
                // 0. Error checking/init
                if (points == null || points.Length < 2) { throw new ArgumentException("Alpha Shapes need at least 2 points"); }
                _segments = new List<Vector2Int>();
                _points = points;
                _alphaCircles = new List<Vector2>();

                _alpha = alpha;

                ComputeCircles();

            }

            private void ComputeCircles()
            {
                float alpha2 = _alpha * _alpha;
                // 1. Run through all pairs of points
                for (int i = 0; i < _points.Length - 1; i++)
                {
                    for (int j = i + 1; j < _points.Length; j++)
                    {
                        if (Vector3.Distance(_points[i], _points[j]) < EPSILON) throw new ArgumentException("AlphaShape needs pairwise distinct points");
                        // 1.1 Find two circles that contains p1 and p2 for each pair p
                        // to find the orthogonal vector, we use the 3D space
                        Vector3 IJ = _points[j] - _points[i];

                        if (IJ.magnitude > 2 * _alpha) continue;

                        Vector2 axis = Vector3.Cross(IJ, Vector3.forward).normalized;
                        Vector2 midPoint = (_points[i] + _points[j]) / 2;

                        float midPointToCenterDist = Mathf.Sqrt(alpha2 - Mathf.Pow(IJ.magnitude / 2, 2));
                        Vector2 C1 = midPoint + midPointToCenterDist * axis;
                        Vector3 C2 = midPoint - midPointToCenterDist * axis;

                        // 1.2 Check if one of the circle is alpha-exposed i.e no other point lies in it
                        bool inC1 = false, inC2 = false;

                        for (int k = 0; k < _points.Length; k++)
                        {
                            if (k == i || k == j) continue;
                            if (Vector2.Distance(_points[k], C1) < _alpha) inC1 = true;
                            if (Vector2.Distance(_points[k], C2) < _alpha) inC2 = true;

                            if (inC1 && inC2) break;
                        }

                        if (!inC1 || !inC2) _segments.Add(new Vector2Int(i, j));
                        if (!inC1) _alphaCircles.Add(C1);
                        if (!inC2) _alphaCircles.Add(C2);
                    }
                }
            }

            public int[] GetContourPointsIndex()
            {
                HashSet<int> res = new HashSet<int>();
                foreach (var segment in _segments)
                {
                    res.Add(segment.x);
                    res.Add(segment.y);
                }
                return res.ToArray();
            }

            public Vector2[] GetContourPointsPosition()
            {
                List<Vector2> res = new List<Vector2>();
                foreach (int index in GetContourPointsIndex()) res.Add(_points[index]);
                return res.ToArray();
            }
        }
    }