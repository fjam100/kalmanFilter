#include <Eigen/Dense>
using namespace Eigen;
// Class definition for matrices
class Model{
	private:
	    MatrixXd M;
	    MatrixXd C;
		MatrixXd A;
		MatrixXd B;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Model(){
			A=MatrixXd::Zero(6,6);
			B=MatrixXd::Zero(6,2);
			M=MatrixXd::Zero(2,2);
			C=MatrixXd::Zero(2,2);
			M << 1, 0,
		        0,0.3;
		    C << 2,0,
		        0,2;
		    A << MatrixXd::Zero(2,2), MatrixXd::Identity(2,2), MatrixXd::Zero(2,2),
		        MatrixXd::Zero(2,2), -M.inverse()*C, -M.inverse(),
		        MatrixXd::Zero(2,6);
		    B << MatrixXd::Zero(2,2),
		        M.inverse(),
		        MatrixXd::Zero(2,2);
		}
		MatrixXd getM();
		MatrixXd getC();	
		MatrixXd getA();
		MatrixXd getB();
};

MatrixXd Model::getM(){
	return M; 
}

MatrixXd Model::getC(){
	return C; 
}

MatrixXd Model::getA(){
	return A;
}

MatrixXd Model::getB(){
 	return B;
}

