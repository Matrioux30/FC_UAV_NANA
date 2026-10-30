// Early proof of concept: fixed state-feedback gain, no QP solver. The full MPC (ADMM) lives in src/ and include/.
#include <iostream>
#include <vector>
#include <iomanip>

using namespace std;

// Parameter Dinamika Sistem (Sesuai dengan MATLAB: Ts = 0.005 s)
const double Ts = 0.005;
const double A[2][2] = {{1.0, Ts}, {0.0, 1.0}};
const double B[2] = {0.5 * Ts * Ts, Ts};

// Batasan Aktuator
const double U_MIN = -15.0;
const double U_MAX = 15.0;

// Fungsi untuk membatasi nilai kendali (Constraint Enforcement)
double apply_constraints(double u) {
    if (u > U_MAX) return U_MAX;
    if (u < U_MIN) return U_MIN;
    return u;
}

int main() {
    // Inisialisasi State (Posisi = 0, Kecepatan = 0)
    double x[2] = {0.0, 0.0};
    double setpoint = 2.0; // Target posisi 2 meter
    
    // Gain Feedback (Disederhanakan dari hasil kalkulasi Cost Function MATLAB)
    // Pada implementasi nyata, nilai ini digantikan oleh Solver Quadratic Programming
    double K[2] = {25.5, 12.2}; 

    int N_steps = 100; // Simulasi 100 iterasi (0.5 detik)

    cout << "====================================================\n";
    cout << " CO-SIMULATION C++ : RESPON TRAJEKTORI UAV \n";
    cout << "====================================================\n";
    cout << setw(10) << "Waktu(s)" << setw(15) << "Posisi(m)" 
         << setw(15) << "Kecepatan" << setw(15) << "Gaya u(N)\n";
    cout << "----------------------------------------------------\n";

    for (int k = 0; k <= N_steps; ++k) {
        double t = k * Ts;

        // 1. Hitung Error (Setpoint - Posisi Aktual)
        double error_pos = setpoint - x[0];
        double error_vel = 0.0 - x[1]; 

        // 2. Kalkulasi Sinyal Kendali (Fungsi Prediksi & Optimasi Disederhanakan)
        double u_raw = (K[0] * error_pos) + (K[1] * error_vel);
        
        // 3. Terapkan Batasan Fisik (Constraints)
        double u_opt = apply_constraints(u_raw);

        // Cetak status saat ini ke terminal
        if (k % 10 == 0) { // Cetak setiap 10 iterasi agar terminal tidak penuh
            cout << fixed << setprecision(3);
            cout << setw(10) << t 
                 << setw(15) << x[0] 
                 << setw(15) << x[1] 
                 << setw(15) << u_opt << "\n";
        }

        // 4. Update State Model Dinamika: x(k+1) = A*x(k) + B*u(k)
        double x_next[2];
        x_next[0] = (A[0][0] * x[0]) + (A[0][1] * x[1]) + (B[0] * u_opt);
        x_next[1] = (A[1][0] * x[0]) + (A[1][1] * x[1]) + (B[1] * u_opt);

        x[0] = x_next[0];
        x[1] = x_next[1];
    }

    cout << "====================================================\n";
    return 0;
}