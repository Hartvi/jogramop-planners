#include <iostream>
#include <functional>
#include <vector>
#include <tuple>
#include <typeinfo>
#include <cstdlib>
#include <cxxabi.h>
#include <Eigen/Dense>

#ifndef BUR_FUNCS_H
#define BUR_FUNCS_H

namespace Burs
{
    using namespace Eigen;
    using ForwardKinematics = std::function<Vector3d(const int &ith_distal_point, const VectorXd &configuration)>;
    using ForwardRt = std::function<std::tuple<std::vector<Matrix3d>, std::vector<Vector3d>>(VectorXd q)>;
    using RadiusFunc = std::function<double(const int &ith_distal_point, const VectorXd &q_k)>;

    using PhiFunc = std::function<double(double)>;

    // // Helper function to demangle C++ type names
    // std::string demangle(const char *name)
    // {
    //     int status = -1;
    //     char *demangled = abi::__cxa_demangle(name, NULL, NULL, &status);
    //     std::string result(status == 0 ? demangled : name);
    //     free(demangled);
    //     return result;
    // }

    // // Factory to produce "not implemented" functions
    // template <typename FuncType>
    // FuncType NotImplementedFunctionFactory()
    // {
    //     return [](auto &&...args) -> decltype(auto)
    //     {
    //         // 1. Print input argument types
    //         std::cout << "Input argument types: ";
    //         (std::cout << ... << demangle(typeid(args).name())) + " ";
    //         std::cout << std::endl;

    //         // 2. Print return type
    //         std::cout << "Return type: " << demangle(typeid(decltype(std::declval<FuncType>()(args...))).name()) << std::endl;

    //         // 3. End the program
    //         std::exit(EXIT_FAILURE);
    //         return decltype(std::declval<FuncType>()(args...)){};
    //     };
    // }
    // template <typename Ret, typename... Args>
    // std::function<Ret(Args...)> NotImplementedFunctionFactory()
    // {
    //     return [](Args... args) -> Ret
    //     {
    //         // 1. Print input argument types
    //         std::cout << "Input argument types: ";
    //         (std::cout << ... << demangle(typeid(args).name()) + " ");
    //         std::cout << std::endl;

    //         // 2. Print return type
    //         std::cout << "Return type: " << demangle(typeid(Ret).name()) << std::endl;

    //         // 3. End the program
    //         std::exit(EXIT_FAILURE);
    //         return Ret{}; // This line will never be executed, but it's needed to make the lambda compile
    //     };
    // }
}
#endif
