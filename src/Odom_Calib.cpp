#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
//typedef Eigen::Matrix<double,3,9> Matrix9d;
typedef Eigen::Matrix<double,1,9> Vector9d_T;
typedef Eigen::Matrix<double,9,1> Vector9d;
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //TODO: 构建超定方程组
        //end of TODO
		Eigen::Matrix<double,3,9> A_tmp;
		Eigen::Matrix<double,3,1> b_tmp;
		A_tmp.setZero();
		b_tmp.setZero();
	        
                b_tmp = scan;

		for(int i = 0; i < 3; i++){
			Vector9d_T tmp;
			tmp.setZero();
			tmp.block<1,3>(0,i*3) = Odom.transpose();
		        A_tmp.block<1,9>(i,0) = tmp;
		}
		A.block<3,9>(now_len*3,0) =  A_tmp;//J^T * J
		b.block<3,1>(now_len*3,0) =  b_tmp;//J^T * B
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;
    //TODO:求解线性最小二乘
    //end of TODO
    Vector9d x;
    Eigen::MatrixXd A_tmp;
    Eigen::VectorXd b_tmp;
    A_tmp = A.transpose() * A;
    b_tmp = A.transpose() * b;
    x = A_tmp.ldlt().solve(b_tmp);//Cholesky decomposition
    correct_matrix << x(0),x(1),x(2),
                      x(3),x(4),x(5),
                      x(6),x(7),x(8);
    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
