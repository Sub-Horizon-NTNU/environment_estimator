#include "KalmanFilter.hpp"


    KalmanFilter::KalmanFilter(const Eigen::VectorXd x, const Eigen::MatrixXd &A,const Eigen::MatrixXd &H,const Eigen::MatrixXd &P)
    : x_(x),A_(A), H_(H), P_(P)
    {
        I_ = Eigen::MatrixXd::Identity(A_.rows(),A_.cols());

        x_pri_.resize(x_.rows(),x_.cols());
        P_pri_.resize(P.rows(),P.cols());

        Q_.resize(A.rows(),A.cols());
        R_.resize(A.rows(),A.cols());
    }

    void KalmanFilter::update(){
        predict();
        correct();
    
    }

    Eigen::VectorXd KalmanFilter::get_predicted_state(const Eigen::MatrixXd &A){
        return A*x_;
    }

    void KalmanFilter::predict(){
        x_pri_ = A_*x_;
        P_pri_ = A_*P_*A_.transpose()+Q_;
        
    }
    
    void KalmanFilter::correct(){
        //Correct measurement
        K_ = P_pri_*H_.transpose()*(H_*P_pri_*H_.transpose()+R_).inverse();
        x_ = x_+K_*(z_-H_*x_pri_);
        P_ = (I_- K_*H_) * P_pri_;
    }
    
    void KalmanFilter::set_model(const Eigen::MatrixXd &A){
        A_ = A; 
    }

    void KalmanFilter::set_measurement(const Eigen::VectorXd &z){
        z_ = z;
    }

    void KalmanFilter::set_process_noise_cov(const Eigen::MatrixXd &Q) {
        Q_ = Q;
    }

    void KalmanFilter::set_measurement_noise_cov(const Eigen::MatrixXd &R){
        R_ = R;
    }

    Eigen::VectorXd KalmanFilter::get_estimates() const{
        return x_;
    }

    Eigen::VectorXd KalmanFilter::get_predictions() const {
        return x_pri_;
    }