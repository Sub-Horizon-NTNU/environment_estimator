
#pragma once
#include <eigen3/Eigen/Dense>

class KalmanFilter{
    public:
    KalmanFilter(const Eigen::VectorXd x, const Eigen::MatrixXd &A,const Eigen::MatrixXd &H,const Eigen::MatrixXd &P);
    
        void update();
        
        Eigen::VectorXd get_predicted_state(const Eigen::MatrixXd &A);

        void predict();

        void correct();
        
        void set_model(const Eigen::MatrixXd &A);
        void set_measurement(const Eigen::VectorXd &z);
        void set_process_noise_cov(const Eigen::MatrixXd &Q);
        void set_measurement_noise_cov(const Eigen::MatrixXd &R);

        Eigen::VectorXd get_estimates() const;
        Eigen::VectorXd get_predictions() const;

    private:
        Eigen::VectorXd x_;
        Eigen::MatrixXd A_;
        //Eigen::MatrixXd B_;
        Eigen::MatrixXd H_;
        Eigen::MatrixXd I_;
        Eigen::VectorXd z_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd R_;
        
        Eigen::MatrixXd K_;
        Eigen::MatrixXd P_pri_;
        Eigen::VectorXd x_pri_;

};