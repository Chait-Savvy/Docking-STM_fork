#ifndef MeanFilter_h
#define MeanFilter_h

template <typename T, int S>
class MeanFilter
{
public:
  /* Constructor */
  MeanFilter() : m_idx(0), m_cnt(0), m_mean(0)
  {
  }

  /* addSample(s): adds the sample S to the window and computes the mean
   * if enough samples have been gathered
   */
  void addSample(T s)
  {
    m_buf[m_idx] = s;
    m_idx = (m_idx + 1) % S;

    if (m_cnt == S)
    {
      p_calcMean();
    }
    else
    {
      m_cnt++;
    }
  }

  /* getMean(): returns the mean computed when the last sample was
   * added. Does not return anything meaningful if not enough samples
   * have been gathered; check isReady() first.
   */
  T getMean()
  {
    return m_mean;
  }

private:
  int m_idx, m_cnt;
  T m_buf[S], m_mean;

  /* p_calcMean(): helper to calculate the mean. Sums all values
   * in the buffer and divides by the number of samples (S).
   */
  void p_calcMean()
  {
    T sum = 0;
    for (int i = 0; i < S; i++)
    {
      sum += m_buf[i];
    }
    //PRINTF("%d\n",m_idx);
    //PRINTF("index %d distance %d %d %d %d %d\n", m_idx, m_buf[0], m_buf[1], m_buf[2], m_buf[3], m_buf[4]);
    m_mean = sum / S;  // Calculate the mean by dividing the sum by the number of samples.
    //PRINTF("mean from function %d\n", m_mean);
  }
};

#endif
