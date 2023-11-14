#ifndef ONE_EURO_FILTER_H_
#define ONE_EURO_FILTER_H_

#include <cmath>
#include <ros/ros.h>

namespace homing_local_planner
{
    class OneEuroFilter
    {
    public:
        OneEuroFilter(){};
        OneEuroFilter(ros::Time t0, double x0, double dx0, double min_cutoff, double beta,
                      double d_cutoff) : t_prev_(t0), x_prev_(x0), dx_prev_(dx0),
                                         min_cutoff_(min_cutoff), beta_(beta), d_cutoff_(d_cutoff){};
        ~OneEuroFilter(){};
        inline double exponentialSmoothing(double a, double x, double x_prev)
        {
            return a * x + (1 - a) * x_prev;
        };
        inline double smoothingFactor(double t_e, double cutoff)
        {
            double r = 2 * M_PI * cutoff * t_e;
            return r / (r + 1.0);
        };
        double filterCall(ros::Time t, double x)
        {
            double t_e = (t - t_prev_).toSec();
            double a_d = smoothingFactor(t_e, dx_prev_);
            double dx = (x - x_prev_) / t_e;
            double dx_hat = exponentialSmoothing(a_d, dx, dx_prev_);
            double cutoff = min_cutoff_ + beta_ * fabs(dx_hat);
            double a = smoothingFactor(t_e, cutoff);
            double x_hat;
            if (fabs(x) > 0.0001)
                x_hat = exponentialSmoothing(a, x, x_prev_);
            else
                x_hat = 0;
            x_prev_ = x_hat;
            dx_prev_ = dx_hat;
            t_prev_ = t;
            return x_hat;
        };

    private:
        double min_cutoff_, beta_, d_cutoff_, x_prev_, dx_prev_;
        ros::Time t_prev_;
    };
};
#endif