namespace TestPolynome
{
    internal class Polynome
    {
        private List<float> _coefficients;

        public float[] Coefficients { get { return _coefficients.ToArray(); } }
        public float this[int key]
        {
            get
            {
                if (key >= _coefficients.Count) return 0;
                return _coefficients[key];
            }

            set
            {
                if (key >= _coefficients.Count)
                {
                    int missingCoeffs = key - _coefficients.Count;
                    for (int i = 0; i <= missingCoeffs; i++) _coefficients.Add(0.0f);
                }
                _coefficients[key] = value;


            }
        }
        public int Degree { get { return _coefficients.Count - 1;} }

        public Polynome()
        {
            _coefficients = new List<float>();
            _coefficients.Add(0.0f);
        }
        public Polynome(float[] coefficients)
        {
            _coefficients = coefficients.ToList();
        }
        public Polynome(float coeffMax, float[] roots)
        {
            Polynome res = new Polynome(new float[1] {coeffMax});
            for (int i = 0; i < roots.Length; i++)
            {
                res *= new Polynome(new float[2] { -roots[i], 1.0f});
            }
            _coefficients = res.Coefficients.ToList();
        }

        public double Evaluate(double x)
        {
            double res = 0;

            for (int i = 0; i < _coefficients.Count; i++)
            {
                if (_coefficients[i] != 0.0f) res += _coefficients[i] * Math.Pow(x, i);
            }

            return res;
        }

        public static Polynome operator *(Polynome A, Polynome B)
        {
            int n = A.Degree + B.Degree;

            float[] coeffs = new float[n+1];

            for (int i = 0; i <= n; i++)
            {
                float ci = 0.0f;
                for (int j = 0; j <= i; j++)
                {
                    ci += A[j] * B[i - j];
                }
                coeffs[i] = ci;
            }

            return new Polynome(coeffs.ToArray());
        }
        public static Polynome operator *(float a, Polynome B)
        {
            if (a == 0.0f) return new Polynome();
            float[] coeffs = new float[B.Coefficients.Length];
            for (int i = 0; i < B.Coefficients.Length; i++) coeffs[i] = a * B[i];
            return new Polynome(coeffs);
        }
        public static Polynome operator *(Polynome a, float b)
        {
            return b * a;
        }
        public static Polynome operator +(Polynome A, Polynome B)
        {
            int n = Math.Max(A.Degree, B.Degree);
            float[] coeffs = new float[n+1];
            for (int i = 0; i <= n; i++)
            {
                coeffs[i] = A[i] + B[i];
            }
            return new Polynome(coeffs);
        }
        public Polynome Derivate(int order)
        {
            if (order == 0) return this;

            Polynome res = this.Derivate();
            for (int i = 2; i <= order; i++)
            {
                res = res.Derivate();
            }
            return res;
        }
        public Polynome Derivate()
        {
            if (this.Degree == 0) return new Polynome();
            float[] coeffs = new float[this.Degree];
            for (int i = 0; i <= this.Degree - 1; i++)
            {
                coeffs[i] = (i + 1) * _coefficients[i+1];
            }

            return new Polynome(coeffs);
        }

        public static Polynome LagrangePolynome(float[] xValues, float[] yValues)
        {
            if (xValues.Length != yValues.Length) throw new System.Exception("X Values and Y Values should be the same size");

            Polynome P = new Polynome();
            int N = xValues.Length;
            //Computing Li 
            for (int i = 0; i < N; i++)
            {
                float invCi = 1.0f;
                Polynome Li = new Polynome(new float[1] { yValues[i] });
                for (int j = 0; j < N; j++)
                {
                    if (i == j) continue;
                    invCi *= (xValues[i] - xValues[j]);
                    Li *= new Polynome(new float[2] { -xValues[j], 1.0f });
                }
                Li *= 1 / invCi;

                P += Li;
            }
            return P;
        }
    }
}
