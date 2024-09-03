#include "KalmanFilterDrone.h"
#include <Arduino.h>

extern SensorData sensorData;
extern bool verbose;
extern float Q_val;
extern float R_val;
extern float P_val;
extern float Pv_val;
// extern const float width;
// extern const float height;
// extern Wall walls[4];
// extern Shape objects[6];
extern State state;

KalmanFilter::KalmanFilter(BLA::Matrix<4, 2> B_init, BLA::Matrix<4, 4> Q_init, BLA::Matrix<4, 4> P_init, BLA::Matrix<4, 1> x_init, float scale_init) {
    B = B_init;
    Q = Q_init;
    P = P_init;
    x = x_init;
    scale = scale_init;
}

void KalmanFilter::predict(BLA::Matrix<4, 4> A, BLA::Matrix<2, 1> u) {
    x = A * x + B * u;
    P = A * P * (~A) + Q;
    if (verbose) {
        Serial.println("After predict:");
        printMatrix(x);
        printMatrix(P);
    }
}

void KalmanFilter::update(BLA::Matrix<1, 1> z, BLA::Matrix<1, 4> H, BLA::Matrix<1, 1> R, BLA::Matrix<4, 4>* rotation_matrix) {
    if (rotation_matrix != NULL) {
        x = (~(*rotation_matrix)) * x;
        P = (~(*rotation_matrix)) * P * (*rotation_matrix);
    }

    BLA::Matrix<1, 1> y = z - H * x;
    BLA::Matrix<1, 1> S = (H * P * (~H) + R);
    //BLA::Matrix<1, 1> S_inv = Invert(S) ;
    S(0,0) = scale/S(0,0);
    BLA::Matrix<4, 1> K = P * (~H) * S / scale;
    x = x + K * y;

    BLA::Matrix<4, 4> I;
    I.Fill(0);
    for (int i = 0; i < 4; i++) {
        I(i, i) = 1;
    }

    P = (I - K * H) * P;

    if (rotation_matrix != NULL) {
        x = (*rotation_matrix) * x;
        P = (*rotation_matrix) * P * (~(*rotation_matrix));
    }
    if (verbose) {
        Serial.println("After update:");
        printMatrix(x);
        printMatrix(P);
    }
}

void KalmanFilter::printMatrix(BLA::Matrix<4, 1> &mat) {
    for (int i = 0; i < 4; i++) {
        Serial.print(mat(i) / scale);
        Serial.print(" ");
    }
    Serial.println();
}

void KalmanFilter::printMatrix(BLA::Matrix<4, 4> &mat) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Serial.print(mat(i, j) / scale);
            Serial.print(" ");
        }
        Serial.println();
    }
}

KalmanHandler::KalmanHandler(BLA::Matrix<4, 1> x, float scale, float width, float height) {
    BLA::Matrix<4, 2> B;
    B.Fill(0);
    B(2, 1) = 1;
    B(3, 0) = 1;

    BLA::Matrix<4, 4> Q;
    Q.Fill(0);
    Q(0, 0) = Q_val;
    Q(1, 1) = Q_val;
    Q(2, 2) = Q_val;
    Q(3, 3) = Q_val;

    BLA::Matrix<4, 4> P;
    P.Fill(0);
    P(0, 0) = P_val;
    P(1, 1) = P_val;
    P(2, 2) = Pv_val;
    P(3, 3) = Pv_val;

    kalman = new KalmanFilter(B, Q, P, x, scale);

    // Initialize walls
    walls[0] = {1, width / 2, .10 * scale, width, 0.10 * scale, 0};       // Top wall
    walls[1] = {2, width / 2, height - .10 * scale, width, 0.10 * scale, 180}; // Bottom wall
    walls[2] = {3, .10 * scale, height / 2, height, 0.10 * scale, 270};   // Left wall
    walls[3] = {4, width - .10 * scale, height / 2, height, 0.10 * scale, 90}; // Right wall

    // Initialize objects
    float base_length = width * 0.185 / 0.300;
    float top_y = height / 4;
    float middle_y = height / 2;
    float bottom_y = height * 3 / 4;

    float left_x_east = width / 2 + base_length / 2;
    float right_x_east = left_x_east - 3.50 * scale;
    float top_x_east = left_x_east;

    float left_x_west = width / 2 - base_length / 2;
    float right_x_west = left_x_west + 3.50 * scale;
    float top_x_west = left_x_west;

    objects[0] = {"circle", left_x_east, bottom_y};
    objects[1] = {"triangle", top_x_east, top_y};
    objects[2] = {"square", right_x_east, middle_y};
    objects[3] = {"circle", left_x_west, bottom_y};
    objects[4] = {"triangle", top_x_west, top_y};
    objects[5] = {"square", right_x_west, middle_y};
}

void KalmanHandler::update(State state, BLA::Matrix<2, 1> u, float dt) {
    float angle = state.angle;
    float distance_to_wall = state.distance_to_wall * kalman->scale;
    int wall_id = state.wall_id;

    BLA::Matrix<4, 4> A;
    A.Fill(0);
    A(0, 0) = 1;
    A(0, 2) = dt;
    A(1, 1) = 1;
    A(1, 3) = dt;

    kalman->predict(A, u);

    for (int i = 0; i < state.shapes_in_range_count; i++) {
        float z, theta;
        BLA::Matrix<1, 4> H;
        BLA::Matrix<1, 1> R;
        object_angle_function(objects[state.shapes_ids[i]], angle, state.angles_to_shapes[i], z, H, R, theta);

        BLA::Matrix<2, 2> rotation_2x2;
        rotation_2x2(0, 0) = cos(theta);
        rotation_2x2(0, 1) = -sin(theta);
        rotation_2x2(1, 0) = sin(theta);
        rotation_2x2(1, 1) = cos(theta);

        BLA::Matrix<4, 4> rotation_matrix;
        rotation_matrix.Fill(0);
        rotation_matrix(0, 0) = rotation_2x2(0, 0);
        rotation_matrix(0, 1) = rotation_2x2(0, 1);
        rotation_matrix(1, 0) = rotation_2x2(1, 0);
        rotation_matrix(1, 1) = rotation_2x2(1, 1);
        rotation_matrix(2, 2) = rotation_2x2(0, 0);
        rotation_matrix(2, 3) = rotation_2x2(0, 1);
        rotation_matrix(3, 2) = rotation_2x2(1, 0);
        rotation_matrix(3, 3) = rotation_2x2(1, 1);

        BLA::Matrix<1, 1> z_matrix;
        z_matrix(0, 0) = z;
        kalman->update(z_matrix, H, R, &rotation_matrix);
    }

    if (wall_id != 0) {
        for (int i = 0; i < 4; i++) {
            if (walls[i].id == wall_id) {
                float z, theta;
                BLA::Matrix<1, 4> H;
                BLA::Matrix<1, 1> R;
                wall_measurement(walls[i], angle, distance_to_wall, z, H, R, theta);

                BLA::Matrix<2, 2> rotation_2x2;
                rotation_2x2(0, 0) = cos(theta);
                rotation_2x2(0, 1) = -sin(theta);
                rotation_2x2(1, 0) = sin(theta);
                rotation_2x2(1, 1) = cos(theta);

                BLA::Matrix<4, 4> rotation_matrix;
                rotation_matrix.Fill(0);
                rotation_matrix(0, 0) = rotation_2x2(0, 0);
                rotation_matrix(0, 1) = rotation_2x2(0, 1);
                rotation_matrix(1, 0) = rotation_2x2(1, 0);
                rotation_matrix(1, 1) = rotation_2x2(1, 1);
                rotation_matrix(2, 2) = rotation_2x2(0, 0);
                rotation_matrix(2, 3) = rotation_2x2(0, 1);
                rotation_matrix(3, 2) = rotation_2x2(1, 0);
                rotation_matrix(3, 3) = rotation_2x2(1, 1);

                BLA::Matrix<1, 1> z_matrix;
                z_matrix(0, 0) = z;
                kalman->update(z_matrix, H, R, &rotation_matrix);
                break;
            }
        }
    }

    // Calculate eigenvalues and eigenvectors of the 2x2 submatrix
    BLA::Matrix<2, 1> eigenvalues;
    BLA::Matrix<2, 2> eigenvectors;
    BLA::Matrix<2, 2> P_2x2;
    P_2x2(0, 0) = kalman->P(0, 0);
    P_2x2(0, 1) = kalman->P(0, 1);
    P_2x2(1, 0) = kalman->P(1, 0);
    P_2x2(1, 1) = kalman->P(1, 1);

    calculateEigenvaluesAndEigenvectors(P_2x2, eigenvalues, eigenvectors);

    // Calculate the rotation angle
    this->angle = atan2(eigenvectors(1, 0), eigenvectors(0, 0)) * 180.0 / PI;
    

    this->eigenvalue0 = eigenvalues(0);
    this->eigenvalue1 = eigenvalues(1);

    if (verbose) {
        Serial.println("After update:");
        for (int i = 0; i < 4; i++) {
            Serial.print(kalman->x(i) / kalman->scale);
            Serial.print(" ");
        }
        Serial.println();
    }
}





float calculate_distance_to_origin(float wall_x, float wall_y, float wall_length, float wall_angle) {
    float wall_angle_rad = radians(wall_angle);
    float wall_end_x = wall_x + wall_length * cos(wall_angle_rad);
    float wall_end_y = wall_y + wall_length * sin(wall_angle_rad);
    float x1 = wall_x;
    float y1 = wall_y;
    float x2 = wall_end_x;
    float y2 = wall_end_y;
    float distance_to_origin = abs((y2 - y1) * 0 - (x2 - x1) * 0 + x2 * y1 - y2 * x1) / sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    return distance_to_origin;
}

void wall_measurement(Wall wall, float compass_measurement, float distance_measurement, float &distance_diff, BLA::Matrix<1, 4> &H, BLA::Matrix<1, 1> &R, float &wall_angle_rad) {
    float distance_to_origin = calculate_distance_to_origin(wall.x, wall.y, wall.length, wall.angle);
    wall_angle_rad = radians(wall.angle);
    float compass_angle_rad = radians(compass_measurement);
    float perpendicular_distance = distance_measurement * sin(wall_angle_rad - compass_angle_rad);
    distance_diff = -distance_to_origin + perpendicular_distance;
    H.Fill(0);
    H(0, 1) = 1;
    R.Fill(0);
    R(0, 0) = R_val;
}

void object_angle_function(Shape object, float compass_measurement, float angle_measurement, float &distance_to_origin, BLA::Matrix<1, 4> &H, BLA::Matrix<1, 1> &R, float &total_angle_rad) {
    float total_angle = compass_measurement + angle_measurement;
    total_angle_rad = radians(total_angle);
    float line_dx = cos(total_angle_rad);
    float line_dy = sin(total_angle_rad);
    float x1 = 0, y1 = 0;
    float x2 = object.x, y2 = object.y;
    distance_to_origin = (line_dx * y2 - line_dy * x2) / sqrt(line_dy * line_dy + line_dx * line_dx);
    H.Fill(0);
    H(0, 1) = 1;
    R.Fill(0);
    R(0, 0) = R_val;
}

void calculateEigenvaluesAndEigenvectors(BLA::Matrix<2, 2>& matrix, BLA::Matrix<2, 1>& eigenvalues, BLA::Matrix<2, 2>& eigenvectors) {
    float a = matrix(0, 0);
    float b = matrix(0, 1);
    float c = matrix(1, 0);
    float d = matrix(1, 1);
    float trace = a + d;
    float det = a * d - b * c;
    float temp = sqrt(sq(trace) - 4 * det);
    eigenvalues(0) = (trace + temp) / 2.0;
    eigenvalues(1) = (trace - temp) / 2.0;
    if (b != 0) {
        eigenvectors(0, 0) = eigenvalues(0) - d;
        eigenvectors(1, 0) = b;
        eigenvectors(0, 1) = eigenvalues(1) - d;
        eigenvectors(1, 1) = b;
    } else if (c != 0) {
        eigenvectors(0, 0) = c;
        eigenvectors(1, 0) = eigenvalues(0) - a;
        eigenvectors(0, 1) = c;
        eigenvectors(1, 1) = eigenvalues(1) - a;
    } else {
        eigenvectors.Fill(0);
        eigenvectors(0, 0) = 1;
        eigenvectors(1, 1) = 1;
    }
    for (int i = 0; i < 2; i++) {
        float norm = sqrt(sq(eigenvectors(0, i)) + sq(eigenvectors(1, i)));
        eigenvectors(0, i) /= norm;
        eigenvectors(1, i) /= norm;
    }
    if (eigenvalues(0) < eigenvalues(1)) {
        float temp_val = eigenvalues(0);
        eigenvalues(0) = eigenvalues(1);
        eigenvalues(1) = temp_val;
        float temp_vec0 = eigenvectors(0, 0);
        float temp_vec1 = eigenvectors(1, 0);
        eigenvectors(0, 0) = eigenvectors(0, 1);
        eigenvectors(1, 0) = eigenvectors(1, 1);
        eigenvectors(0, 1) = temp_vec0;
        eigenvectors(1, 1) = temp_vec1;
    }
}
