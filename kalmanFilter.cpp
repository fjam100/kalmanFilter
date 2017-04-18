#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class Kalman{
	int a;	
};


//Pseudo Inverse -> Just in case src: https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

// Class definition for matrices
/*
class Model{
	private:
		MatrixXd A;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Model(){
			MatrixXd M(2,2);
			M << 1, 0,
				0,0.3;
			MatrixXd C(2,2);
			C << 1,0,
				0,1;
			A << MatrixXd::Zero(2,2), MatrixXd::Identity(2,2), MatrixXd::Zero(2,2),
				MatrixXd::Zero(2,2), -M.inverse()*C, -M.inverse(),
				MatrixXd::Zero(2,6);
		}
		MatrixXd getA();		
};

MatrixXd Model::getA(){
	return A;
}
*/

int main(){

	// Model m;
	MatrixXd M(2,2);
	MatrixXd C(2,2);
	MatrixXd A(6,6);
	MatrixXd B(6,2);
	MatrixXd Ad(6,6);
	MatrixXd Bd(6,2);
	// Kalman parameters
	MatrixXd P(6,6);
	MatrixXd Q(6,6);
	MatrixXd R(4,4);
	MatrixXd H(4,6);
	MatrixXd K(6,4);
	// simulation parameter
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
	M << 1, 0,
		0,0.3;	
	C << 2,0,
		0,2;
	A << MatrixXd::Zero(2,2), MatrixXd::Identity(2,2), MatrixXd::Zero(2,2),
		MatrixXd::Zero(2,2), -M.inverse()*C, M.inverse(),
		MatrixXd::Zero(2,6);
	B << MatrixXd::Zero(2,2),
		M.inverse(),
		MatrixXd::Zero(2,2);
	Ad = MatrixXd::Identity(6,6)+A*Ts;
	Bd = B*Ts;
	
	P = MatrixXd::Zero(6,6);
	H << MatrixXd::Identity(4,4), MatrixXd::Zero(4,2);
	Q = MatrixXd::Identity(6,6)*0.01; //Not very certain about model
	R = MatrixXd::Identity(4,4)*0.0000001; //Pretty certain about measurements	
	

	for (int i=0; i < 7/Ts; i++){
		acc = M.inverse()*(U+Fext-C*Xact.tail(2));
		vel = vel+acc*Ts;
		pos = pos+vel*Ts;
		Xact << pos,
			vel;
		
		// Prediction step
		Xest = Ad*Xest+Bd*U;
		P = Ad*P*Ad.transpose()+Q;

		// Measurement update
		K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
		Xest = Xest+K*(Xact-H*Xest);
		P = (MatrixXd::Identity(6,6)-K*H)*P;	
		
	}

	cout << "Est" << Xest << endl;
	cout << "True" << Xact << endl;

	
}
