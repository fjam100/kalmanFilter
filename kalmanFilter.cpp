#include <iostream>
#include <Eigen/Dense>
#include "Model.h"
#include "Kalman.h"
using namespace Eigen;
using namespace std;


//Pseudo Inverse -> Just in case src: https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


int main(){
    Model m;
    Kalman k;
    MatrixXd Ad(6,6);
	MatrixXd Bd(6,2);
	double Ts = 0.001;
	// State
	VectorXd Xest(6);
	Xest << -0.1595, 
 		-0.1987, 
		0, 
		0, 
		0, 
		0; // state estimate with force tacked on
	Vector4d Xact(-0.1595, -0.1987, 0, 0); // actual state
	Vector2d acc(0, 0);
	Vector2d vel(0, 0);
	Vector2d pos(-0.1595, -0.1987);	
	// Input
	Vector2d U(0, 0);
	Vector2d Fext(3.5, 0);

	// setting values
	Ad = MatrixXd::Identity(6,6)+m.getA()*Ts;
	Bd = m.getB()*Ts;
	

	for (int i=0; i < 7/Ts; i++){
		acc = m.getM().inverse()*(U+Fext-m.getC()*Xact.tail(2));
		vel = vel+acc*Ts;
		pos = pos+vel*Ts;
		Xact << pos,
			vel;
		
		// Prediction step
		Xest = Ad*Xest+Bd*U;
		k.setP(Ad*k.getP()*Ad.transpose()+k.getQ());
    
		// Measurement update
		k.setK(k.getP()*k.getH().transpose()*(k.getH()*k.getP()*k.getH().transpose()+k.getR()).inverse());
		Xest = Xest+k.getK()*(Xact-k.getH()*Xest);
		k.setP((MatrixXd::Identity(6,6)-k.getK()*k.getH())*k.getP());	
		
	}

	cout << "Est" << Xest << endl;
	cout << "True" << Xact << endl;
	
}
