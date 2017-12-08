#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <math.h>

int main()
{
    Eigen::Matrix3f R_1_to_3_on_1, R_2_to_3_on_2, R_1_to_2_on_1;
    Eigen::Vector3f t_1_to_3_on_1, t_2_to_3_on_2, t_1_to_2_on_1;

    R_2_to_3_on_2 << 0.5, 0.0, 0.866, 0., 1.0, 0., -0.866, 0., 0.5;
    R_1_to_2_on_1 << 0.9239, 0., 0.3827, 0, 1.0, 0., -0.3827, 0., 0.9239;

    R_1_to_3_on_1 = R_1_to_2_on_1 * R_2_to_3_on_2;

    t_1_to_2_on_1 << 0.02, 0.0, 0.05;
    t_2_to_3_on_2 << 0.03, 0.0, 0.03;

    t_1_to_3_on_1 = t_1_to_2_on_1 + R_1_to_2_on_1*t_2_to_3_on_2;

    Eigen::Vector3f normal_1_3, normal_1_2, normal_2_3;

    double scale1 = sqrt(t_1_to_2_on_1.transpose() * t_1_to_2_on_1);
    double scale2 = sqrt(t_2_to_3_on_2.transpose() * t_2_to_3_on_2);
    double scale3 = sqrt(t_1_to_3_on_1.transpose() * t_1_to_3_on_1);

    normal_1_2 = t_1_to_2_on_1/scale1;
    normal_2_3 = t_2_to_3_on_2/scale2;
    normal_1_3 = t_1_to_3_on_1/scale3;



    Eigen::Vector3f normal_2_3_on_1;
    normal_2_3_on_1 = R_1_to_2_on_1 * normal_2_3;

    Eigen::MatrixXf A(3,2);

    A << normal_1_2(0), normal_2_3_on_1(0),
         normal_1_2(1), normal_2_3_on_1(1),
         normal_1_2(2), normal_2_3_on_1(2);

    Eigen::VectorXf p_Solve= A.jacobiSvd(Eigen::ComputeThinU
                            |Eigen::ComputeThinV).solve(normal_1_3);

    std::cout << "compare Scale: \n" <<  p_Solve << std::endl;

    std::cout << "Scale 1 to 2: " << scale1/scale3
              << "\n Scale 2 to 3: " << scale2/scale3 << std::endl;


    return 0;


}
