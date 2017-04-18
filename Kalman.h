#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
class Kalman{
    private:
        MatrixXd P;
	    MatrixXd Q;
	    MatrixXd R;
	    MatrixXd H;
	    MatrixXd K;
	 
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Kalman(){
            P=MatrixXd::Zero(6,6);
            Q=MatrixXd::Zero(6,6);
	        R=MatrixXd::Zero(4,4);
	        H=MatrixXd::Zero(4,6);
	        K=MatrixXd::Zero(6,4);
	        
	        P = MatrixXd::Zero(6,6);
	        H << MatrixXd::Identity(4,4), MatrixXd::Zero(4,2);
	        Q = MatrixXd::Identity(6,6)*0.01; //Not very certain about model
	        R = MatrixXd::Identity(4,4)*0.0000001; //Pretty certain about measurements	
        }
        void setP(MatrixXd input);
        void setQ(MatrixXd input);
        void setR(MatrixXd input);
        void setH(MatrixXd input);
        void setK(MatrixXd input);
        MatrixXd getP();
        MatrixXd getQ();
        MatrixXd getR();
        MatrixXd getH();
        MatrixXd getK();
              
};

void Kalman::setP(MatrixXd input){
    P=input;
}
void Kalman::setQ(MatrixXd input){
    Q=input;
}
void Kalman::setR(MatrixXd input){
    R=input;
}
void Kalman::setH(MatrixXd input){
    H=input;
}
void Kalman::setK(MatrixXd input){
    K=input;
}
MatrixXd Kalman::getP(){
    return P;
}
MatrixXd Kalman::getQ(){
    return Q;
}
MatrixXd Kalman::getR(){
    return R;
}
MatrixXd Kalman::getH(){
    return H;
}
MatrixXd Kalman::getK(){
    return K;
}
