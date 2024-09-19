#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

void  dataReader( std::vector<double>& x_coordinates, std::vector<double>& y_coordinates, std::vector<std::vector<double>>& z_values, std::vector<std::vector<double>>& stiffness_values, std::vector<std::vector<double>>& damping_values){
    std::string directory = "/home/robotics/ur3_ros2/matlab/data_body/";
    std::string x_filename = directory + "x.txt";
    std::string y_filename = directory + "y.txt";
    std::string z_filename = directory + "z.txt";
    std::string stiffness_filename = directory + "stiffness.txt";
    std::string damping_filename = directory + "damping.txt";

    // Read x coordinates from the first file
    std::ifstream x_file(x_filename);
    double x_value;
    while (x_file >> x_value) {
        x_coordinates.push_back(x_value);
    }
    x_file.close();

    // Read y coordinates from the second file
    std::ifstream y_file(y_filename);
    double y_value;
    while (y_file >> y_value) {
        y_coordinates.push_back(y_value);
    }
    y_file.close();

    // Determine the dimensions (n and m) based on the sizes of x and y vectors
    int n = x_coordinates.size();
    int m = y_coordinates.size();

    // Read z values from the third file and store them in a 2D vector
    std::ifstream z_file(z_filename);
    z_values.resize(n, std::vector<double>(m));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            z_file >> z_values[i][j];
            z_values[i][j] += 0.0015;
        }
    }
    z_file.close();

    // Read stiffness values from the fourth file and store them in a 2D vector
    std::ifstream stiffness_file(stiffness_filename);
    stiffness_values.resize(n, std::vector<double>(m));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            stiffness_file >> stiffness_values[i][j];
        }
    }
    stiffness_file.close();

    // Read damping values from the fifth file and store them in a 2D vector
    std::ifstream damping_file(damping_filename);
    damping_values.resize(n, std::vector<double>(m));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            damping_file >> damping_values[i][j];
        }
    }
    damping_file.close();

    // Now you have x_coordinates, y_coordinates, z_values, stiffness_values, and damping_values
}

int findClosestIndex(const std::vector<double>& vec, double target) {
    int index = 0;
    double minDistance = std::abs(vec[0] - target);

    for (long unsigned int i = 1; i < vec.size(); ++i) {
        double distance = std::abs(vec[i] - target);
        if (distance < minDistance) {
            minDistance = distance;
            index = i;
        }
    }

    return index;
}