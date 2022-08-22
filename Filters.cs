namespace Filters
{
    public class ButterworthFilter
    {

        public enum PassType
        {
            HighPass,
            LowPass,

        }

        private readonly float c, a1, a2, a3, b1, b2;

        /// <summary>
        /// Array of input values, latest are in front
        /// </summary>
        private float[] inputHistory = new float[2];

        /// <summary>
        /// Array of output values, latest are in front
        /// </summary>
        private float[] outputHistory = new float[3];

        /// <summary>
        /// rez amount, from sqrt(2) to ~ 0.1
        /// </summary>
        public ButterworthFilter(float frequency, int sampleRate, PassType passType, float resonance)
        {
            switch (passType)
            {
                case PassType.LowPass:
                    c = 1.0f / (float)Mathf.Tan(Mathf.PI * frequency / sampleRate);
                    a1 = 1.0f / (1.0f + resonance * c + c * c);
                    a2 = 2f * a1;
                    a3 = a1;
                    b1 = 2.0f * (1.0f - c * c) * a1;
                    b2 = (1.0f - resonance * c + c * c) * a1;
                    break;
                case PassType.HighPass:
                    c = (float)Mathf.Tan(Mathf.PI * frequency / sampleRate);
                    a1 = 1.0f / (1.0f + resonance * c + c * c);
                    a2 = -2f * a1;
                    a3 = a1;
                    b1 = 2.0f * (c * c - 1.0f) * a1;
                    b2 = (1.0f - resonance * c + c * c) * a1;
                    break;
            }
        }
        public void Update(float newInput)
        {
            float newOutput = a1 * newInput + a2 * this.inputHistory[0] + a3 * this.inputHistory[1] - b1 * this.outputHistory[0] - b2 * this.outputHistory[1];

            this.inputHistory[1] = this.inputHistory[0];
            this.inputHistory[0] = newInput;

            this.outputHistory[2] = this.outputHistory[1];
            this.outputHistory[1] = this.outputHistory[0];
            this.outputHistory[0] = newOutput;
        }

        public float Value { get { return this.outputHistory[0]; } }


    }
    public class MeanFilter
    {
        private int _size;
        private Queue<float> _values;

        public float[] Values { get { return _values.ToArray(); } }
        public int Size { get { return _size; } }
        public float Value { get; private set; }

        public bool IsFilled { get; private set; }
        public MeanFilter(int size)
        {
            _size = size;
            _values = new Queue<float>();
            Value = 0;
            IsFilled = false;
        }

        public void Update(float input)
        {
            float fractionnedInput = input / _size;
            _values.Enqueue(fractionnedInput);
            Value += fractionnedInput;
            if (_values.Count > _size)
            {
                if (!IsFilled) IsFilled = true;
                Value -= _values.Peek();
                _values.Dequeue();
            }
        }
    } 
}

