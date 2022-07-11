

#include <ceres/ceres.h>

#include <chrono>
// #include <eigen3/Eigen/Eigen>
#include <iostream>

class CURVE_FITTING_COST {
   private:
    double _x, _y;

   public:
    template <typename T>
    bool operator()(const T* const abcd, T* residual) const {
        residual[0] = T(_y) - (abcd[0] * (T(_x)) - abcd[1] * ceres::exp(T(_x)) + abcd[2] * ceres::pow(T(_x), 3) + abcd[3]);
        return true;
    }

    CURVE_FITTING_COST(double x, double y) {
        _x = x;
        _y = y;
    }
    // ~CURVE_FITTING_COST() {
    //     std::cout << "cost function finish.." << std::endl;
    // }
};
// TODO : 다른 nonlinear 함수 만들어서 변수 어려개 input 값 여러개 , noise추가해서
int main(int argc, char const* argv[]) {
    // a * x * x + b * x + c
    double a = 1.5, b = 4.5, c = 6.6, d = 1.0;  // correct result
    std::vector<double> x_data, y_data;
    int N = 100;
    // double abcd[4];
    // abcd << 0.0, 0.0, 0.0, 0.0;
    double abcd[4] = {0.0, 0.0, 0.0, 0.0};
    /****** complete generating data******/

    for (uint8_t i = 0; i < N; i++) {
        double t = i * 0.01;
        x_data.push_back(t);
        y_data.push_back(a * (t)-b * std::exp(t) + c * std::pow(t, 3) + d);
        std::cout << x_data[i] << ", " << y_data[i] << std::endl;
    }

    /****ceres start*****/
    ceres::Problem problem;

    /***residual***/
    // TODO : y = a*log(x) - b*exp(x)+ c*x**3+d

    for (int i = 0; i < N; i++) {
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(new CURVE_FITTING_COST(x_data[i], y_data[i]));
        problem.AddResidualBlock(cost_function, nullptr, abcd);
    }

    ceres::Solver::Options options;
    // options.max_num_iterations = 900;
    // options.preconditioner_type = ceres::JACOBI;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.nonlinear_conjugate_gradient_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // std::cout << abc << std::endl;
    std::cout << summary.BriefReport() << std::endl;

    for (auto& p : abcd) std::cout << p << std::endl;

    return 0;
}

// class CURVE_FITTING_COST {
//    private:
//     double _x, _y;

//    public:
//     template <typename T>
//     bool operator()(const T* const abc, T* residual) const {
//         residual[0] = T(_y) - (abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
//         return true;
//     }

//     CURVE_FITTING_COST(double x, double y) {
//         _x = x;
//         _y = y;
//     }
//     // ~CURVE_FITTING_COST() {
//     //     std::cout << "cost function finish.." << std::endl;
//     // }
// };

// ceres::Solver::Options options;
// options.linear_solver_type = ceres::DENSE_QR;
// options.minimizer_progress_to_stdout = true;

// ceres::Solver::Summary summary;
// chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
// ceres::Solve(options, &problem, &summary);
// chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
// chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
// std::cout << "solve time = " << time_used.count() << endl;

// std::cout << summary.BriefReport() << std::endl;
// std::cout << "a,b,c = ";
// for (auto& p : abc) std::cout << p << " " << std::endl;

// ceres::Problem problem;
// for (int i = 0; i < N; i++) {
//     problem.AddResidualBlock(
//         new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
//             new CURVE_FITTING_COST(x_data[i], y_data[i])),
//         nullptr, abc);
// }

#if 0
#include <ceres/ceres.h>

#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;

struct CURVE_FITTING_COST {
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
    template <typename T>
    bool operator()(const T* const abc, T* residual) const {
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }
    const double _x, _y;
};

int main(int argc, char const* argv[]) {
    double a = 1.0, b = 2.0, c = 1.0;
    int N = 100;
    double w_sigma = 1.0;
    cv::RNG rng;
    double abc[3] = {0, 0, 0};

    std::vector<double> x_data, y_data;

    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        std::cout << x_data[i] << ", " << y_data[i] << std::endl;
    }

    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])),
            nullptr, abc);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    std::cout << "solve time = " << time_used.count() << endl;

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "a,b,c = ";
    for (auto& p : abc) std::cout << p << " " << std::endl;

    return 0;
}

#endif
