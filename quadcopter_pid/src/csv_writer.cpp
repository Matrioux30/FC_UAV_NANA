#include "csv_writer.h"
#include <iomanip>
#include <limits>

namespace quad {

CsvWriter::CsvWriter(const std::string& filename) {
    file_.open(filename);
    file_ << std::setprecision(std::numeric_limits<double>::max_digits10);
}

CsvWriter::~CsvWriter() {
    close();
}

void CsvWriter::writeHeader() {
    if (!file_.is_open()) return;
    file_ << "time,x_ref,y_ref,z_ref,psi_ref,x,y,z,phi,theta,psi,p,q,r,ex,ey,ez,F_total,tau_x,tau_y,tau_z,omega1,omega2,omega3,omega4\n";
}

void CsvWriter::writeRow(const LogRow& row) {
    if (!file_.is_open()) return;
    file_ << row.time << ","
          << row.x_ref << "," << row.y_ref << "," << row.z_ref << "," << row.psi_ref << ","
          << row.x << "," << row.y << "," << row.z << ","
          << row.phi << "," << row.theta << "," << row.psi << ","
          << row.p << "," << row.q << "," << row.r << ","
          << row.ex << "," << row.ey << "," << row.ez << ","
          << row.F_total << "," << row.tau_x << "," << row.tau_y << "," << row.tau_z << ","
          << row.omega1 << "," << row.omega2 << "," << row.omega3 << "," << row.omega4 << "\n";
}

void CsvWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

} // namespace quad
