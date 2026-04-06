
#pragma once
#include <eigen3/Eigen/Dense>

class KalmanFilter{
    public:
    KalmanFilter(const Eigen::VectorXd x, const Eigen::MatrixXd &A,const Eigen::MatrixXd &H,const Eigen::MatrixXd &P)
    : x_(x),A_(A), H_(H), P_(P)
    {
        I_ = Eigen::MatrixXd::Identity(A_.rows(),A_.cols());

        x_pri_.resize(x_.rows(),x_.cols());
        P_pri_.resize(P.rows(),P.cols());

        Q_.resize(A.rows(),A.cols());
        R_.resize(A.rows(),A.cols());
    }

    void set_model(const Eigen::MatrixXd &A){
        A_ = A; 
    }

    void set_measurement(const Eigen::VectorXd &z){
        z_ = z;
    }

    void set_process_noise_cov(const Eigen::MatrixXd &Q) {
        Q_ = Q;
    }

    void set_measurement_noise_cov(const Eigen::MatrixXd &R){
        R_ = R;
    }
    

    void update(){
        //TODO ADD dynamically updating const velocity prediction model 
        //Predict 
        predict();
        correct();
        
        //Correct measurement
    }

    void predict(){
        x_pri_ = A_*x_;
        P_pri_ = A_*P_*A_.transpose()+Q_;

    }

    void correct(){
        //Correct measurement
        K_ = P_pri_*H_.transpose()*(H_*P_pri_*H_.transpose()+R_).inverse();
        x_ = x_+K_*(z_-H_*x_pri_);
        P_ = (I_- K_*H_) * P_pri_;
    }

    Eigen::VectorXd get_estimates(){
        return x_;
    }
    Eigen::VectorXd get_predictions(){
        return x_pri_;
    }

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