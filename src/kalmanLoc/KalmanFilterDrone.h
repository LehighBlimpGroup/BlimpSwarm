#ifndef KALMAN_FILTER_DRONE_H
#define KALMAN_FILTER_DRONE_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

#define CONTROL_PARAMS_SIZE (13 * sizeof(float))
#define STATE_INFO_SIZE (sizeof(float) * 6 + sizeof(int) * 7)

struct ControlInput {
    float params[13];
};

struct SensorData {
    float values[6];
};

struct State {
    float angle;
    float distance_to_wall;
    int wall_id;
    int shapes_in_range_count;
    int shapes_ids[4];
    float angles_to_shapes[4];
    int test;
};

struct Wall {
    int id;
    float x, y, length, width, angle;
};

struct Shape {
    String shape;
    float x, y;
};

class KalmanFilter {
  public:
    BLA::Matrix<4, 1> x; // State estimate
    BLA::Matrix<4, 4> P; // Estimate error covariance
    BLA::Matrix<4, 4> Q; // Process noise covariance
    BLA::Matrix<4, 2> B; // Control input matrix
    float scale; // Scale factor

    KalmanFilter(BLA::Matrix<4, 2> B_init, BLA::Matrix<4, 4> Q_init, BLA::Matrix<4, 4> P_init, BLA::Matrix<4, 1> x_init, float scale_init);
    void predict(BLA::Matrix<4, 4> A, BLA::Matrix<2, 1> u);
    void update(BLA::Matrix<1, 1> z, BLA::Matrix<1, 4> H, BLA::Matrix<1, 1> R, BLA::Matrix<4, 4>* rotation_matrix = NULL);

  private:
    void printMatrix(BLA::Matrix<4, 1> &mat);
    void printMatrix(BLA::Matrix<4, 4> &mat);
};

class KalmanHandler {
  public:
    KalmanFilter* kalman;
    float eigenvalue0;
    float eigenvalue1;
    float angle;
    Wall walls[4];
    Shape objects[6];

    KalmanHandler(BLA::Matrix<4, 1> x, float scale, float width, float height);
    void update(State state, BLA::Matrix<2, 1> u, float dt);
};





float calculate_distance_to_origin(float wall_x, float wall_y, float wall_length, float wall_angle);
void wall_measurement(Wall wall, float compass_measurement, float distance_measurement, float &distance_diff, BLA::Matrix<1, 4> &H, BLA::Matrix<1, 1> &R, float &wall_angle_rad);
void object_angle_function(Shape object, float compass_measurement, float angle_measurement, float &distance_to_origin, BLA::Matrix<1, 4> &H, BLA::Matrix<1, 1> &R, float &total_angle_rad);
void calculateEigenvaluesAndEigenvectors(BLA::Matrix<2, 2>& matrix, BLA::Matrix<2, 1>& eigenvalues, BLA::Matrix<2, 2>& eigenvectors);

#endif
