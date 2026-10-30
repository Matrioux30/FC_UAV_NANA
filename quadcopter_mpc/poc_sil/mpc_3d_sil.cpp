// Early proof of concept: per-axis PD on a point mass, no QP solver. The full MPC (ADMM) lives in src/ and include/.
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

// Parameter Dinamika
const double Ts = 0.05;
const double Kp = 2.0; // Gain posisi
const double Kd = 1.5; // Gain kecepatan

// Fungsi utama
int main() {
    ofstream outFile("trajectory_sil.csv");
    outFile << "t,X,Y,Z\n"; // Header CSV

    // State awal (Posisi dan Kecepatan untuk X, Y, Z)
    double x[3] = {0.0, 0.0, 0.0}; // Posisi X, Y, Z
    double v[3] = {0.0, 0.0, 0.0}; // Kecepatan X, Y, Z

    int N_steps = 200; // 10 detik

    for (int k = 0; k <= N_steps; ++k) {
        double t = k * Ts;

        // Simpan data posisi ke CSV
        outFile << t << "," << x[0] << "," << x[1] << "," << x[2] << "\n";

        // Target Trajektori (Heliks 3D)
        double ref_X = 5.0 * cos(t);
        double ref_Y = 5.0 * sin(t);
        double ref_Z = 0.5 * t;
        
        // Target kecepatan (Turunan dari target posisi)
        double ref_vX = -5.0 * sin(t);
        double ref_vY = 5.0 * cos(t);
        double ref_vZ = 0.5;

        // Kontroler Prediktif Sederhana untuk tiap sumbu (Menghitung Gaya/U)
        double u_X = Kp * (ref_X - x[0]) + Kd * (ref_vX - v[0]);
        double u_Y = Kp * (ref_Y - x[1]) + Kd * (ref_vY - v[1]);
        double u_Z = Kp * (ref_Z - x[2]) + Kd * (ref_vZ - v[2]);

        // Update Dinamika Sistem (Posisi & Kecepatan baru)
        x[0] = x[0] + (Ts * v[0]) + (0.5 * Ts * Ts * u_X);
        v[0] = v[0] + (Ts * u_X);

        x[1] = x[1] + (Ts * v[1]) + (0.5 * Ts * Ts * u_Y);
        v[1] = v[1] + (Ts * u_Y);

        x[2] = x[2] + (Ts * v[2]) + (0.5 * Ts * Ts * u_Z);
        v[2] = v[2] + (Ts * u_Z);
    }

    outFile.close();
    cout << "Simulasi C++ selesai! Data disimpan ke 'trajectory_sil.csv'" << endl;
    return 0;
}