#ifndef KALMANFILTER
#define KALMANFILTER

#include <Eigen/Core>
#include <Eigen/LU>

class KalmanFilter{
    public:
        KalmanFilter(){}
        ~KalmanFilter(){}
        void init(double _dt){
            dt = _dt;
            A << 1, dt, 0, 1;
            H << 1, 0;
            Q << 0.001, 0, 0, 0.001;
            R << 0.25;
            I << 1, 0, 0, 1;
            x << 0, 1;
            P << 1, 0, 0, 1;
        }
        double process(double input){
            Eigen::Matrix<double, 1, 1> z;
            Eigen::Matrix<double, 2, 1> K;
            z << input;

            x = A * x;
            P = A * P * A.transpose() + Q;

            K = P * H.transpose() *  (H * P * H.transpose() + R).inverse();
            x = x + K * (z - H * x);
            P = (I - K * H) * P;

            return x(0, 0);
        }

    private:
        double dt;
        Eigen::Matrix<double, 2, 2> A;
        Eigen::Matrix<double, 1, 2> H;
        Eigen::Matrix<double, 2, 2> Q;
        Eigen::Matrix<double, 1, 1> R;
        Eigen::Matrix<double, 2, 2> I;

        Eigen::Matrix<double, 2, 1> x;
        Eigen::Matrix<double, 2, 2> P;

};



#endif