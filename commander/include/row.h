#pragma once

struct row{
    double r;
    double theta;
    bool operator < (row rhs){
        return(r < rhs.r);
    };
    row operator += (row rhs){
        row ret;
        ret.r = r + rhs.r;
        ret.theta = theta + rhs.theta;

        return ret;
    };
    row operator /= (double rhs){
        row ret;
        ret.r = r / rhs;
        ret.theta = r/rhs;
        return ret;
    }
};