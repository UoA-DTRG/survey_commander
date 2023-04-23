#pragma once

#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

class outlier_filter{
public:
    outlier_filter(int size, double gate){
        for(size_t i = 0; i < size; ++i){
            m_samples.push_back(0);
        }
        m_initialized = false;
        m_size = size;
        m_head = 0;
        m_gate = gate;
    };

    bool add_sample(double sample){
        m_samples[m_head] = sample;
        m_head++;
        if(!m_initialized && m_head <= m_size){
            if(m_head >= m_size){
                m_initialized = true;
            }else{
                return false;
            }
        }

        // If we're here then the sample accumulator is full
        // Reset the circular buffer pointer if it's dead
        if(m_head >= m_size) m_head -= m_size;

        m_cache_valid = false;

        double mean = calc_mean();
        double sd = calc_sd(mean);

        double dist = (sample - mean) / sd;

        printf("Got sample: %f, mean: %f, sd: %f, gate_size: %f, dist: %f\n", sample, mean, sd, m_gate, dist);
        
        if(dist < m_gate){
            return true;
        }

        return false;
    };

    bool check_sample(double sample){
        if(!m_initialized && m_head <= m_size){
            if(m_head >= m_size) m_initialized = true;

            return false;
        }

        if(!m_cache_valid){
            m_mean_cache = calc_mean();
            m_sd_cache = calc_sd(m_mean_cache);
            m_cache_valid = true;
        }

        double dist = (sample - m_mean_cache) / m_sd_cache;

        printf("Check sample: %f, mean: %f, sd: %f, gate_size: %f, dist: %f\n", sample, m_mean_cache, m_sd_cache, m_gate, dist);
        
        if(dist < m_gate){
            return true;
        }

        return false;
    }

private:
    int m_size;
    int m_head;
    double m_gate;
    vector<double> m_samples;
    bool m_initialized;

    double m_sd_cache;
    double m_mean_cache;
    bool m_cache_valid;

    double calc_mean(){
        double mean = 0;
        for(auto s : m_samples){
            mean += s;
        }

        return mean / ((double) m_samples.size());
    }

    double calc_sd(double mean){
        double variance = 0;
        for(auto s : m_samples){
            variance += (s - mean) * (s - mean);
        }
        variance /= (double) m_size;

        // FIXME This is a hack
        if(variance <= 0.01) return 0.01;

        return sqrt(variance);
    }
};