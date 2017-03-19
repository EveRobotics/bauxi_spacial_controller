

/*
 Cumulative moving average[edit]
 In a cumulative moving average, the data arrive in an ordered datum stream,
 and the user would like to get the average of all of the data up until the
 current datum point. For example, an investor may want the average price of
 all of the stock transactions for a particular stock up until the current
 time. As each new transaction occurs, the average price at the time of the
 transaction can be calculated for all of the transactions up to that point
 using the cumulative average, typically an equally weighted average of the
 sequence of n values x_1. \ldots, x_n up to the current time:
 
 The brute-force method to calculate this would be to store all of the data
 and calculate the sum and divide by the number of datum points every time a
 new datum point arrived. However, it is possible to simply update cumulative
 average as a new value, x_{n+1} becomes available, using the formula:
 
 \textit{CMA}_{n+1} = {{x_{n+1} + n \cdot \textit{CMA}_n} \over {n+1}}
 Thus the current cumulative average for a new datum point is equal to the
 previous cumulative average, times n, plus the latest datum point, all
 divided by the number of points received so far, n+1. When all of the datum
 points arrive (n = N), then the cumulative average will equal the final
 average.
 
 The derivation of the cumulative average formula is straightforward. Using
 
 x_1 + \cdots + x_n = n \cdot \textit{CMA}_n
 and similarly for n + 1, it is seen that
 
 x_{n+1} = (x_1 + \cdots + x_{n+1}) - (x_1 + \cdots + x_n) = (n+1) \cdot
 \textit{CMA}_{n+1} - n \cdot \textit{CMA}_n
 Solving this equation for \textit{CMA}_{n+1} results in:
 
 \textit{CMA}_{n+1} = {x_{n+1} + n \cdot \textit{CMA}_n \over {n+1}}
 = {\textit{CMA}_n} + {{x_{n+1} - \textit{CMA}_n} \over {n+1}}
 */

class MovingAverage {
    
public:
    // Perhaps we can have a non-pure virtual function here.
    virtual void addSample(float sample) {
        _sampleBuffer[_currentSample] = sample;
        //Serial.print("Add Sample at index: ");
        //Serial.println(_currentSample);
        _currentSample = (_currentSample + 1) % _bufferSize;
        // Increment the sample count until it equals the buffer size:
        if(_sampleCount < _bufferSize) {
            _sampleCount++;
        }
        //Serial.print("Add Sample, count: ");
        //Serial.println(_sampleCount);
    }
    
    virtual float computeAverage(void) = 0;
    
    virtual ~MovingAverage() {};

protected:
    float *_sampleBuffer;
    unsigned int _bufferSize;
    unsigned int _currentSample;
    unsigned int _sampleCount;
    
private:

};

class CumulativeMovingAverage : public MovingAverage {
    
public:
    
private:
    
};


/*
 * Weighted moving average implementation, used for sensor
 * data averaging.
 */
class WeightedMovingAverage : public MovingAverage {
    
public:
    
    WeightedMovingAverage(unsigned int bufferSize) {
        _sampleBuffer = new float[bufferSize];
        _bufferSize = bufferSize;
        _sampleCount = 0;
    }
    
    //
    float computeAverage(void) {
        double average = 0.0;
        unsigned int multSum = 0;
        int index = _currentSample - 1;
        if(index < 0) {
            index = (int)_sampleCount - 1;
        }
        for(int i = 0; i < (int)_sampleCount; i++) {
            average += _sampleBuffer[index] * ((int)_sampleCount - i);
            // Work from the most recent sample to the oldest sample.
            index = index - 1;
            if(index < 0) {
                index = (int)_sampleCount - 1;
            }
            multSum += (unsigned int)((int)_sampleCount - i);
        }
        return (float)(average / (double)multSum);
    }
    
private:
    
};



/*
 Exponential moving average[edit]

 An exponential moving average (EMA), also known as an exponentially weighted 
 moving average (EWMA),[5] is a type of infinite impulse response filter that 
 applies weighting factors which decrease exponentially. The weighting for 
 each older datum decreases exponentially, never reaching zero. The graph at 
 right shows an example of the weight decrease.
 
 The EMA for a series Y may be calculated recursively:
 
 S_1  = Y_1
 for  t > 1,\ \    S_{t} = \alpha \cdot Y_{t} + (1-\alpha) \cdot S_{t-1}
 
 Where:
 
 The coefficient α represents the degree of weighting decrease, a constant 
 smoothing factor between 0 and 1. A higher α discounts older observations faster.
 
 Yt is the value at a time period t.
 
 St is the value of the EMA at any time period t.
 */

class ExponentialMovingAverage : public MovingAverage {
    
public:
    
    ExponentialMovingAverage(void) {
        _alpha = 0.5;
        _average = 0.0;
    }

    ExponentialMovingAverage(float alpha) {
        _alpha = alpha;
        _average = 0.0;
    }

    virtual void addSample(float sample) {
        if(_sampleCount == 0) {
            _sampleCount++;
            _average = sample;
        } else {
            _average = _alpha * sample + (1.0 - _alpha) * _average;
        }
        
    }
    
    void setAlpha(float alpha) {
        _alpha = alpha;
    }

    float computeAverage(void) {
        return _average;
    }
    
private:
    float _alpha = 0.5;
    float _average = 0.0;
    
};




