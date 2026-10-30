#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <fstream>
#include <string>
#include <vector>

namespace quad {

struct LogRow {
    double time;
    double x_ref, y_ref, z_ref, psi_ref;
    double x, y, z;
    double phi, theta, psi;
    double p, q, r;
    double ex, ey, ez;
    double F_total, tau_x, tau_y, tau_z;
    double omega1, omega2, omega3, omega4;
};

class CsvWriter {
public:
    explicit CsvWriter(const std::string& filename);
    ~CsvWriter();

    void writeHeader();
    void writeRow(const LogRow& row);
    void close();

private:
    std::ofstream file_;
};

} // namespace quad

#endif
