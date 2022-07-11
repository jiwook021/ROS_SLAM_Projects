 #include <ceres/ceres.h>
#include <iostream>
#include <stdio.h>

#include <vector>


class CURVE_FITTING_COST{
    private:
        double _x, _y;

    public:

    template<typename T>
    bool operator() (const T* const abc, T* residual) const {
        //residual[0] = T(_y) - abc[0] *T(_x)*T(_x)+abc[1]*T(_x)+abc[2];
        
        residual[0] = T(_y) - (abc[0]*(T(_x)) - abc[1]*ceres::exp(T(_x)) + abc[2]* ceres::pow(T(_x),3) + abc[3]);
        //   y_data.push_back(a*t - b*std::exp(t) + c*t*t*t + d);
 
       // a*t-b*std::exp(t)+c*t*t*t+d

        return true;
    }
        CURVE_FITTING_COST(double x, double y){
            _x = x;
            _y = y;
        }
        ~CURVE_FITTING_COST()
        {
     //       std::cout << "Cost function finish.." <<std::endl;
        }
};
//TODO: try different nonlinear functions with many different variables, noise  
int main()
{
    
    // (a * x * x + b*x +c)
    //(y = a*log(x) - b*exp(x) + c*x*x*x)
    double a = 1.5;
    double b = 4.5;
    double c= 6.6;
    double d= 1.0;
    
    double abc[4]={0,0,0,0};

    std::vector<double> x_data,y_data;

    int N = 100;
    
    for(int i = 0; i<N ;i++){
        double t = (double) i/N;
        x_data.push_back(t);
        y_data.push_back(a*(t) - b*std::exp(t) + c*std::pow(t,3) + d);
    }
    
    /****complete generating data *****/

    /**solve problem with ceres***/
    ceres::Problem problem;
    for (int i = 0; i<N; i++){
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                new CURVE_FITTING_COST(x_data[i],y_data[i])),nullptr,abc
            );
        }
    ceres::Solver::Options options;
    //options.max_num_iterations = 300;
    //options.linear_solver_type = ceres::DENSE_QR;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    
    //options.nonlinear_conjugate_gradient_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true; 

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout<<summary.BriefReport()<<std::endl;
    for (auto &p : abc) std::cout << p <<std::endl;

    return 0;
}