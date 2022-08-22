namespace BezierCalculations
{
    public class Vector3
    {
        public float x { get; set; }
        public float y { get; set; }
        public float z { get; set; }

        public float magnitude { get { return MathF.Sqrt(sqrMagnitude); } }
        public float sqrMagnitude { get { return x * x + y * y + z * z; } }
        public Vector3 normalized
        {
            get
            {
                float mag = magnitude;
                return new Vector3(x / mag, y / mag, z / mag);
            }
        }

        public Vector3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Vector3 operator *(float a, Vector3 vec)
        {
            return new Vector3(vec.x * a, vec.y * a, vec.z * a);
        }
        public static Vector3 operator *(Vector3 vec, float a)
        {
            return a * vec;
        }

        public static Vector3 operator /(Vector3 vec, float a)
        {
            return (1 / a) * vec;
        }

        public static Vector3 operator +(Vector3 A, Vector3 B)
        {
            return new Vector3(A.x + B.x, A.y + B.y, A.z + B.z);
        }

        public static Vector3 operator -(Vector3 A, Vector3 B)
        {
            return new Vector3(A.x - B.x, A.y - B.y, A.z - B.z);
        }

        public static float Distance(Vector3 A, Vector3 B)
        {
            return (A - B).magnitude;
        }

        public static float Angle(Vector3 A, Vector3 B)
        {
            float num = (float)Math.Sqrt(A.sqrMagnitude * B.sqrMagnitude);
            if (num < 1E-15f)
            {
                return 0f;
            }
            float num2 = Math.Clamp(Dot(A, B), -1f, 1f);
            return (float)Math.Acos(num2) * 57.29578f;
        }

        public static float Dot(Vector3 A, Vector3 B)
        {
            return A.x * B.x + A.y * B.y + A.z * B.z;
        }
    }

    public static class CubicBezierTools
    {
        // https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf

        static public Vector3 Curve(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3)
        {
            return
                (1 - t) * (1 - t) * (1 - t) * P0 +
                3 * t * (1 - t) * (1 - t) * P1 +
                3 * t * t * (1 - t) * P2 +
                t * t * t * P3;
        }
        static public Vector3 CurveDerivate1(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3)
        {
            return
                -3 * (1 - t) * (1 - t) * P0 +
                (1 - t) * (3 - 9 * t) * P1 +
                t * (6 - 9 * t) * P2 +
                3 * t * t * P3;
        }
        static public Vector3 CurveDerivate2(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3)
        {
            return
                6 * (1 - t) * P0 +
                6 * (3 * t - 2) * P1 +
                6 * (1 - 3 * t) * P2 +
                6 * t * P3;
        }
        static public Vector3 CurveDerivate3(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3)
        {
            return
                -6 * P0 +
                18 * P1 +
                -18 * P2 +
                6 * P3;
        }
        static public float CurvatureRadius(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3)
        {
            Vector3 Bp = CurveDerivate1(t, P0, P1, P2, P3);
            Vector3 Bpp = CurveDerivate2(t, P0, P1, P2, P3);

            double k = (Bp.x * Bpp.z - Bp.z * Bpp.x) / Math.Pow((Bp.x * Bp.x + Bp.z * Bp.z), 3.0f / 2.0f);
            return 1 / (float)k;
        }
        static public float FastLength(Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3)
        {
            float chord = (P3 - P0).magnitude;
            float cont_net = (P1 - P0).magnitude + (P2 - P1).magnitude + (P3 - P2).magnitude;
            return (chord + cont_net) / 2;
        }

        static public CubicBezierCurve[] SolveC2CubicBezierRaccord(Vector3[] controlPoints)
        {
            int n = controlPoints.Length;
            float[] bArray = new float[n - 1];
            float[] cArray = new float[n - 1];
            float[] aArray = new float[n - 1];
            Vector3[] dArray = new Vector3[n - 1];
            Vector3[] P1Array = new Vector3[n - 1];
            Vector3[] P2Array = new Vector3[n - 1];
            float[] cpArray = new float[n - 1];
            Vector3[] dpArray = new Vector3[n - 1];

            for (int i = 0; i < n - 1; i++)
            {
                bArray[i] = 4.0f;
                cArray[i] = 1.0f;
                aArray[i] = 1.0f;
                dArray[i] = 4.0f * controlPoints[i] + 2.0f * controlPoints[i + 1];
            }
            bArray[0] = 2.0f;
            bArray[n - 2] = 7.0f;

            aArray[n - 2] = 2.0f;

            dArray[0] = controlPoints[0] + 2.0f * controlPoints[1];
            dArray[n - 2] = 8 * controlPoints[n - 2] + controlPoints[n - 1];

            //Calculating cp
            cpArray[0] = cArray[0] / bArray[0];
            for (int i = 1; i < n - 1; i++)
            {
                cpArray[i] = cArray[i] / (bArray[i] - aArray[i] * cpArray[i - 1]);
            }

            //Calculating dp
            dpArray[0] = dArray[0] / bArray[0];
            for (int i = 1; i < n - 1; i++)
            {
                dpArray[i] = (dArray[i] - aArray[i] * dpArray[i - 1]) / (bArray[i] - aArray[i] * cpArray[i - 1]);

            }

            //Calculating P1
            P1Array[n - 2] = dpArray[n - 2];
            for (int i = 1; i < n - 1; i++)
            {
                int index = (n - 2) - i;
                P1Array[index] = dpArray[index] - cpArray[index] * P1Array[index + 1];
            }

            for (int i = 0; i < n - 2; i++)
            {
                P2Array[i] = 2 * controlPoints[i + 1] - P1Array[i + 1];
            }
            P2Array[n - 2] = 0.5f * (controlPoints[n - 1] + P1Array[n - 2]);

            List<CubicBezierCurve> curvesC2 = new List<CubicBezierCurve>();
            for (int i = 0; i < n - 1; i++)
            {
                CubicBezierCurve curve = new CubicBezierCurve(
                    controlPoints[i],
                    P1Array[i],
                    P2Array[i],
                    controlPoints[i + 1]);
                curvesC2.Add(curve);
            }
            return curvesC2.ToArray();
        }

        static public CubicBezierCurve[] SolveC2CubicBezierRaccord(Vector3[] controlPoints, Vector3 derivate1, Vector3 derivate2)
        {
            int n = controlPoints.Length;
            CubicBezierCurve[] curves = new CubicBezierCurve[n - 1];

            curves[0] = new CubicBezierCurve(
                controlPoints[0],
                controlPoints[0] + derivate1 / 3,
                controlPoints[0] + 2 / 3 * derivate1 + derivate2 / 6,
                controlPoints[1]);

            for (int i = 1; i < curves.Length; i++)
            {
                Vector3 P1i = 2 * controlPoints[i] - curves[i - 1].P2;
                Vector3 P2i = 2 * controlPoints[i] + curves[i - 1].P1 - 4 * curves[i - 1].P2;
                curves[i] = new CubicBezierCurve(
                    controlPoints[i],
                    P1i,
                    P2i,
                    controlPoints[i + 1]);
            }
            return curves;
        }
    }

    public static class QuinticBezierTools
    {
        static public Vector3 Curve(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, Vector3 P4, Vector3 P5)
        {
            float ImT = 1 - t;
            float ImT2 = (1 - t) * ImT;
            float ImT3 = (1 - t) * ImT2;
            float ImT4 = (1 - t) * ImT3;
            float ImT5 = (1 - t) * ImT4;

            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;
            float t5 = t4 * t;

            return
                ImT5 * P0 +
                5 * t * ImT4 * P1 +
                10 * t2 * ImT3 * P2 +
                10 * t3 * ImT2 * P3 +
                5 * t4 * ImT * P4 +
                t5 * P5;
        }

        static public Vector3 CurveDerivate1(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, Vector3 P4, Vector3 P5)
        {
            float ImT = 1 - t;
            float ImT2 = (1 - t) * ImT;
            float ImT3 = (1 - t) * ImT2;
            float ImT4 = (1 - t) * ImT3;

            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;

            return
                -5 * ImT4 * P0 +
                5 * ImT3 * (1 - 5 * t) * P1 +
                10 * t * ImT2 * (2 - 5 * t) * P2 +
                10 * t2 * ImT * (3 - 5 * t) * P3 +
                5 * t3 * (4 - 5 * t) * P4 +
                5 * t4 * P5;
        }

        static public Vector3 CurveDerivate2(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, Vector3 P4, Vector3 P5)
        {
            float ImT = 1 - t;
            float ImT2 = (1 - t) * ImT;
            float ImT3 = (1 - t) * ImT2;

            float t2 = t * t;
            float t3 = t2 * t;

            return
                20 * ImT3 * P0 +
                20 * ImT2 * (-2 + 5 * t) * P1 +
                20 * ImT * (10 * t2 - 8 * t + 1) * P2 +
                20 * t * (10 * t2 - 12 * t + 3) * P3 +
                20 * t2 * (3 - 5 * t) * P4 +
                20 * t3 * P5;
        }

        static public QuinticBezierCurve CurveWithC2LimitConditions(Vector3 A, Vector3 AD1, Vector3 AD2, Vector3 B, Vector3 BD1, Vector3 BD2)
        {
            Vector3 P0 = A;
            Vector3 P1 = A + AD1 / 5.0f;
            Vector3 P2 = A + (2.0f / 5.0f) * AD1 + 1.0f / 20.0f * AD2;
            Vector3 P3 = B - (2.0f / 5.0f) * BD1 + 1.0f / 20.0f * BD2;
            Vector3 P4 = B - BD1 / 5;
            Vector3 P5 = B;

            return new QuinticBezierCurve(P0, P1, P2, P3, P4, P5);
        }

        static public QuinticBezierCurve RaccordC2Curves(C2Curve ACurve, C2Curve BCurve)
        {
            Vector3 A = ACurve.EvaluateT(1.0f);
            Vector3 AD1 = ACurve.EvaluateDerivate1T(1.0f);
            Vector3 AD2 = ACurve.EvaluateDerivate2T(1.0f);


            Vector3 B = BCurve.EvaluateT(0.0f);
            Vector3 BD1 = BCurve.EvaluateDerivate1T(0.0f);
            Vector3 BD2 = BCurve.EvaluateDerivate2T(0.0f);

            return CurveWithC2LimitConditions(A, AD1, AD2, B, BD1, BD2);
        }
    }

    public static class SepticBezierTools
    {
        static public Vector3 Curve(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, Vector3 P4, Vector3 P5, Vector3 P6, Vector3 P7)
        {
            float ImT = 1 - t;
            float ImT2 = (1 - t) * ImT;
            float ImT3 = (1 - t) * ImT2;
            float ImT4 = (1 - t) * ImT3;
            float ImT5 = (1 - t) * ImT4;
            float ImT6 = (1 - t) * ImT5;
            float ImT7 = (1 - t) * ImT6;

            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;
            float t5 = t4 * t;
            float t6 = t5 * t;
            float t7 = t6 * t;

            return
                ImT7 * P0 +
                7 * t * ImT6 * P1 +
                21 * t2 * ImT5 * P2 +
                35 * t3 * ImT4 * P3 +
                35 * t4 * ImT3 * P4 +
                21 * t5 * ImT2 * P5 +
                7 * t6 * ImT * P6 +
                t7 * P7;
        }

        static public Vector3 CurveDerivate1(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, Vector3 P4, Vector3 P5, Vector3 P6, Vector3 P7)
        {
            float ImT = 1 - t;
            float ImT2 = (1 - t) * ImT;
            float ImT3 = (1 - t) * ImT2;
            float ImT4 = (1 - t) * ImT3;
            float ImT5 = (1 - t) * ImT4;
            float ImT6 = (1 - t) * ImT5;

            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;
            float t5 = t4 * t;
            float t6 = t5 * t;

            return
                -7 * ImT6 * P0 +
                ImT5 * (7 - 49 * t) * P1 +
                21 * t * ImT4 * (2 - 7 * t) * P2 +
                t2 * ImT3 * (105 - 245 * t) * P3 +
                t3 * ImT2 * (140 - 245 * t) * P4 +
                21 * t4 * ImT * (5 - 7 * t) * P5 +
                t5 * (42 - 49 * t) * P6 +
                7 * t6 * P7;

        }

        static public Vector3 CurveDerivate2(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, Vector3 P4, Vector3 P5, Vector3 P6, Vector3 P7)
        {
            float ImT = 1 - t;
            float ImT2 = (1 - t) * ImT;
            float ImT3 = (1 - t) * ImT2;
            float ImT4 = (1 - t) * ImT3;
            float ImT5 = (1 - t) * ImT4;

            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;
            float t5 = t4 * t;

            return
                42 * ImT5 * P0 +
                ImT4 * (294 * t - 84) * P1 +
                42 * ImT3 * (10 * t2 - 10 * t * ImT + ImT2) * P2 +
                210 * t * (7 * t4 - 20 * t3 + 20 * t2 - 8 * t + 1) * P3 +
                t2 * (-1470 * t3 + 3150 * t2 - 2100 * t + 420) * P4 +
                t3 * (882 * t2 - 1260 * t + 420) * P5 +
                t4 * (210 - 294 * t) * P6 +
                42 * t5 * P7;

        }

        static public Vector3 CurveDerivate3(float t, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, Vector3 P4, Vector3 P5, Vector3 P6, Vector3 P7)
        {
            float ImT = 1 - t;
            float ImT2 = (1 - t) * ImT;
            float ImT3 = (1 - t) * ImT2;
            float ImT4 = (1 - t) * ImT3;

            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;

            return
                -210 * ImT4 * P0 +
                ImT3 * (630 - 1470 * t) * P1 +
                (-4410 * t4 + 12600 * t3 - 12600 * t2 + 5040 * t - 630) * P2 +
                (7350 * t4 - 16800 * t3 + 12600 * t2 - 3360 * t + 210) * P3 +
                210 * t * (-35 * t3 + 60 * t2 - 30 * t + 4) * P4 +
                t2 * (4410 * t2 - 5040 * t + 1260) * P5 +
                t3 * (840 - 1470 * t) * P6 +
                210 * t4 * P7;
        }

        static public SepticBezierCurve RaccordWithC3LimitConditions(Vector3 A, Vector3 AD1, Vector3 AD2, Vector3 AD3, Vector3 B, Vector3 BD1, Vector3 BD2, Vector3 BD3)
        {
            Vector3 P0 = A;
            Vector3 P7 = B;

            Vector3 P1 = P0 + AD1 / 7.0f;
            Vector3 P6 = P7 - BD1 / 7.0f;

            Vector3 P2 = -1 * P0 + 2 * P1 + AD2 / 42.0f;
            Vector3 P5 = 2 * P6 - P7 + BD2 / 42.0f;

            Vector3 P3 = P0 - 3 * P1 + 3 * P2 + AD3 / 210.0f;
            Vector3 P4 = 3 * P5 - 3 * P6 + P7 - BD3 / 210.0f;

            return new SepticBezierCurve(P0, P1, P2, P3, P4, P5, P6, P7);

        }

        static public SepticBezierCurve RaccordC3Curves(C3Curve ACurve, C3Curve BCurve)
        {
            Vector3 A = ACurve.EvaluateT(1.0f);
            Vector3 AD1 = ACurve.EvaluateDerivate1T(1.0f);
            Vector3 AD2 = ACurve.EvaluateDerivate2T(1.0f);
            Vector3 AD3 = ACurve.EvaluateDerivate3T(1.0f);


            Vector3 B = BCurve.EvaluateT(0.0f);
            Vector3 BD1 = BCurve.EvaluateDerivate1T(0.0f);
            Vector3 BD2 = BCurve.EvaluateDerivate2T(0.0f);
            Vector3 BD3 = BCurve.EvaluateDerivate3T(0.0f);

            return RaccordWithC3LimitConditions(A, AD1, AD2, AD3, B, BD1, BD2, BD3);
        }
    }

    public abstract class C2Curve
    {
        #region Protected Attributes
        protected int _resolution;
        protected float[] _arcLengths;

        #endregion

        public float CurveLength { get { return _arcLengths.Last(); } }
        public int Resolution
        {
            get => _resolution;
            set
            {
                _resolution = value;
                ComputeArcLengths();
            }
        }

        public abstract Vector3 EvaluateT(float t);
        public abstract Vector3 EvaluateDerivate1T(float t);
        public abstract Vector3 EvaluateDerivate2T(float t);

        #region Order 0
        public Vector3 EvaluateS(float s) //Use arc curvature
        {
            return EvaluateWithCurveLength(s * CurveLength);
        }
        public Vector3 EvaluateWithCurveLength(float length)
        {
            return EvaluateT(MappingFunc(length));
        }
        #endregion
        #region Order 1
        public Vector3 EvaluateDerivate1WithCurveLength(float length) { return EvaluateDerivate1T(MappingFunc(length)); }
        public Vector3 EvaluateDerivate1S(float s) { return EvaluateDerivate1WithCurveLength(s * CurveLength); }
        #endregion
        #region Order 2       
        public Vector3 EvaluateDerivate2WithCurveLength(float length) { return EvaluateDerivate2T(MappingFunc(length)); }
        public Vector3 EvaluateDerivate2S(float s) { return EvaluateDerivate2WithCurveLength(s * CurveLength); }
        #endregion
        #region Curvature Radius
        public float EvaluateCurvatureRadiusT(float t) 
        {
            Vector3 Bp = EvaluateDerivate1T(t);
            Vector3 Bpp = EvaluateDerivate2T(t);
            double k = (Bp.x * Bpp.z - Bp.z * Bpp.x) / Math.Pow((Bp.x * Bp.x + Bp.z * Bp.z), 3.0f / 2.0f);
            return 1 / (float)k;
        }
        public float EvaluateCurvatureRadiusWithLength(float length) { return EvaluateCurvatureRadiusT(MappingFunc(length)); }
        public float EvaluateCurvatureRadiusS(float s) { return EvaluateCurvatureRadiusWithLength(s * CurveLength); }
        #endregion
        public float MappingFunc(float s)
        {
            int left = 0;
            int right = _arcLengths.Length - 1;

            if (s == 0) return 0.0f;
            if (s >= _arcLengths[right]) return 1.0f;

            while (right - left > 1)
            {
                int middle = (left + right) / 2;
                if (_arcLengths[middle] < s)
                {
                    left = middle;
                }
                else if (_arcLengths[middle] > s)
                {
                    right = middle;
                }
                else
                {
                    return (float)middle / (float)_arcLengths.Length;
                }
            }

            float rt = (float)right / (float)_resolution;
            float lt = (float)left / (float)_resolution;

            return (rt - lt) * (s - _arcLengths[left]) / (_arcLengths[right] - _arcLengths[left]) + lt;
        }

        #region Protected Methods
        protected void ComputeArcLengths()
        {
            _arcLengths = new float[_resolution + 1];
            _arcLengths[0] = 0;
            float tm = 0.0f;
            for (int i = 1; i <= _resolution; i++)
            {
                float t = (float)i / (float)_resolution;
                _arcLengths[i] = Vector3.Distance(EvaluateT(t), EvaluateT(tm)) + _arcLengths[i - 1];
                tm = t;
            }
        }
        #endregion
    }

    public abstract class C3Curve : C2Curve
    {
        public abstract Vector3 EvaluateDerivate3T(float t);
        public Vector3 EvaluateDerivate3WithCurveLength(float length) { return EvaluateDerivate3T(MappingFunc(length)); }
        public Vector3 EvaluateDerivate3S(float s) { return EvaluateDerivate3WithCurveLength(s * CurveLength); }

    }

    public class CubicBezierCurve : C3Curve
    {
        private Vector3 _P0, _P1, _P2, _P3;

        #region Public Properties
        public Vector3 P0
        {
            get => _P0;
            set
            {
                _P0 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P1
        {
            get => _P1;
            set
            {
                _P1 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P2
        {
            get => _P2;
            set
            {
                _P2 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P3
        {
            get => _P3;
            set
            {
                _P3 = value;
                ComputeArcLengths();
            }
        }
        #endregion

        #region Constructors
        public CubicBezierCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {

            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            Resolution = 50;
        }
        public CubicBezierCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, int resolution)
        {

            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            Resolution = resolution;
        }
        #endregion

        public override Vector3 EvaluateT(float t) { return CubicBezierTools.Curve(t, P0, P1, P2, P3); }
        public override Vector3 EvaluateDerivate1T(float t) { return CubicBezierTools.CurveDerivate1(t, P0, P1, P2, P3); }
        public override Vector3 EvaluateDerivate2T(float t) { return CubicBezierTools.CurveDerivate2(t, P0, P1, P2, P3); }
        public override Vector3 EvaluateDerivate3T(float t) { return CubicBezierTools.CurveDerivate3(t, P0, P1, P2, P3); }

        public void UpdatePoints(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            ComputeArcLengths();
        }
    }

    public class QuinticBezierCurve : C2Curve
    {
        private Vector3 _P0, _P1, _P2, _P3, _P4, _P5;

        public Vector3 P0
        {
            get => _P0;
            set
            {
                _P0 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P1
        {
            get => _P1;
            set
            {
                _P1 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P2
        {
            get => _P2;
            set
            {
                _P2 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P3
        {
            get => _P3;
            set
            {
                _P3 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P4
        {
            get => _P4;
            set
            {
                _P4 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P5
        {
            get => _P5;
            set
            {
                _P5 = value;
                ComputeArcLengths();
            }
        }

        public QuinticBezierCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, Vector3 p5)
        {

            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            _P4 = p4;
            _P5 = p5;
            Resolution = 50;
        }
        public QuinticBezierCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, Vector3 p5, int resolution)
        {

            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            _P4 = p4;
            _P5 = p5;
            Resolution = resolution;
        }

        public override Vector3 EvaluateT(float t) { return QuinticBezierTools.Curve(t, _P0, _P1, _P2, _P3, _P4, _P5); }

        public override Vector3 EvaluateDerivate1T(float t) { return QuinticBezierTools.CurveDerivate1(t, _P0, _P1, _P2, _P3, _P4, _P5); }

        public override Vector3 EvaluateDerivate2T(float t) { return QuinticBezierTools.CurveDerivate2(t, _P0, _P1, _P2, _P3, _P4, _P5); }

        public void UpdatePoints(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, Vector3 p5)
        {
            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            _P4 = p4;
            _P5 = p5;
            ComputeArcLengths();
        }
    }

    public class SepticBezierCurve : C3Curve
    {
        private Vector3 _P0, _P1, _P2, _P3, _P4, _P5, _P6, _P7;

        public Vector3 P0
        {
            get => _P0;
            set
            {
                _P0 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P1
        {
            get => _P1;
            set
            {
                _P1 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P2
        {
            get => _P2;
            set
            {
                _P2 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P3
        {
            get => _P3;
            set
            {
                _P3 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P4
        {
            get => _P4;
            set
            {
                _P4 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P5
        {
            get => _P5;
            set
            {
                _P5 = value;
                ComputeArcLengths();
            }
        }
        public Vector3 P6
        {
            get => _P6;
            set
            {
                _P6 = value;
                ComputeArcLengths();
            }
        }

        public Vector3 P7
        {
            get => _P7;
            set
            {
                _P7 = value;
                ComputeArcLengths();
            }
        }

        public SepticBezierCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, Vector3 p5, Vector3 p6, Vector3 p7)
        {

            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            _P4 = p4;
            _P5 = p5;
            _P6 = p6;
            _P7 = p7;
            Resolution = 50;
        }
        public SepticBezierCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, Vector3 p5, Vector3 p6, Vector3 p7, int resolution)
        {
            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            _P4 = p4;
            _P5 = p5;
            _P6 = p6;
            _P7 = p7;
            Resolution = resolution;
        }

        public override Vector3 EvaluateT(float t) { return SepticBezierTools.Curve(t, _P0, _P1, _P2, _P3, _P4, _P5, _P6, _P7); }

        public override Vector3 EvaluateDerivate1T(float t) { return SepticBezierTools.CurveDerivate1(t, _P0, _P1, _P2, _P3, _P4, _P5, _P6, _P7); }

        public override Vector3 EvaluateDerivate2T(float t) { return SepticBezierTools.CurveDerivate2(t, _P0, _P1, _P2, _P3, _P4, _P5, _P6, _P7); }

        public override Vector3 EvaluateDerivate3T(float t) { return SepticBezierTools.CurveDerivate3(t, _P0, _P1, _P2, _P3, _P4, _P5, _P6, _P7); }

        public void UpdatePoints(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, Vector3 p5, Vector3 p6, Vector3 p7)
        {
            _P0 = p0;
            _P1 = p1;
            _P2 = p2;
            _P3 = p3;
            _P4 = p4;
            _P5 = p5;
            _P6 = p6;
            _P7 = p7;
            ComputeArcLengths();
        }
    }
}