# Tahapan Implementasi MPC Accelerator di KV260 (Orang Kedua)

Panduan langkah demi langkah dari RTL sampai program berjalan di board. Detail teknis (mikroarsitektur, register map) ada di [README.md](README.md).

**Posisi saat ini:**

| # | Tahap | Status |
|---|---|---|
| 0 | Persiapan tool | Icarus Verilog + Python sudah ada di laptop ini. Vivado belum |
| 1 | Cek lisensi XCK26 | **Belum**, perlu PC dengan Vivado 2022.1 |
| 2 | Mikroarsitektur | Selesai |
| 3 | RTL | Selesai (`hdl/`) |
| 4 | Simulasi vs test vector | **Selesai**: 4000/4000 bit-exact untuk Q16.16 dan Q8.24 |
| 5 | Block design + bitstream | Script siap, belum dijalankan |
| 6 | SD card Ubuntu + boot board | Belum |
| 7 | Load bitstream + program PS | File siap, menunggu tahap 5–6 |
| 8 | Profiling di board + bahan BAB VII | Belum |

Urutan prioritas sambil menunggu hasil profiling orang pertama: **1 → 4 → 5**. Tahap 6 boleh dikerjakan paralel kapan saja, karena tidak bergantung pada bitstream.

---

## Tahap 0: Persiapan tool

**Laptop ini (simulasi):**
- Icarus Verilog sudah terpasang di `C:\msys64\ucrt64\bin`. Jalankan perintah simulasi dari **Git Bash**, lalu tambahkan dulu ke PATH:
  ```bash
  export PATH=/c/msys64/ucrt64/bin:$PATH
  ```
- Python 3 (cukup standard library).
- Opsional: GTKWave untuk melihat waveform (`pacman -S mingw-w64-ucrt-x86_64-gtkwave`).

**PC untuk Vivado:**
- Vivado **2022.1**, dengan device *Zynq UltraScale+ MPSoC* ikut diinstal.
- Ruang disk ≥ 100 GB dan RAM ≥ 16 GB.
- **Penting:** salin folder `mpc_accel_hw/` ke path **tanpa spasi**, misalnya `C:\work\mpc_accel_hw`. Path `Semester 5` akan membuat build gagal.
  Folder `quadcopter_mpc/` tidak perlu ikut disalin, karena semua file yang dibutuhkan sudah ada di `mpc_accel_hw/mem/`.

**Hardware:** KV260 + adaptor power 12 V, microSD ≥ 32 GB, kabel micro-USB (UART/JTAG), kabel Ethernet ke router/PC yang sama jaringannya.

---

## Tahap 1: Verifikasi lisensi XCK26 (prasyarat synthesis)

1. Di PC Vivado, buka **Vivado 2022.1 Tcl Shell**, lalu `cd C:/work/mpc_accel_hw`.
2. Jalankan:
   ```
   vivado -mode batch -source vivado/check_license.tcl
   ```
3. Baca baris terakhir output:

| Output | Arti | Tindakan |
|---|---|---|
| `LICENSE CHECK: PASS` | Synthesis + place ke XCK26 bisa dilakukan | Lanjut ke tahap 5 |
| `part ... not installed` | Device belum diinstal | Installer Xilinx → *Add Design Tools or Devices* → centang *Zynq UltraScale+ MPSoC* |
| Error `Common 17-345` / `17-349` (license) | Lisensi tidak ditemukan | *Vivado License Manager* → *Obtain License* → pilih lisensi Vivado ML Standard (gratis) → load file `.lic` |

4. Screenshot hasil PASS dan halaman *View License Status* di License Manager sebagai bukti untuk laporan.

> K26 seharusnya didukung edisi gratis (Standard), tetapi status ini tetap harus dibuktikan dengan script di atas, bukan diasumsikan.

---

## Tahap 2: Mikroarsitektur (selesai, cukup dipahami)

Pemetaan blok Gambar 7.1 ke RTL:

| Blok Gambar 7.1 | File / op |
|---|---|
| AXI4-Lite Slave Interface | `hdl/mpc_accel_axi.v` |
| Control FSM | `hdl/mpc_core.v` |
| BRAM | ROM koefisien 112 × 640 bit (MX, MR, W) + buffer input x, r |
| Prediction Engine (MAC paralel) | 20 lane, `g = MX·x − MR·r` (92 siklus) |
| Cost Function Unit | op `RHS`: `rho·(z − y) − g` |
| Numerical Solver Kernel | op `MAC` atas W (20 siklus) + `alpha·u + (1−alpha)·z` |
| Constraint Evaluation | op `UPD`: clamp ke ZMIN/ZMAX + update y |

Angka kunci untuk laporan:
- Latensi **2738 siklus = 27,38 µs @ 100 MHz = 0,55 % dari Ts (5 ms)**, rumusnya `33·N_ITER + 98`.
- Latensi deterministik: jitter di sisi PL = 0. Bandingkan dengan profiling CPU yang punya jitter peak-peak 70 µs.
- Estimasi resource: ~80 DSP48E2, ~9 BRAM36, ~15–20 k LUT (dikonfirmasi di tahap 5).

---

## Tahap 3: RTL (selesai)

File utama ada di `hdl/`. RTL ini **sama untuk Q16.16 dan Q8.24**. Yang membedakan keduanya hanya file ROM (`mem/<fmt>/`) dan parameter `FRAC_*`.

**Soal pilihan format (catatan orang pertama poin 3):** biaya hardware keduanya identik (sama-sama 32-bit, sama-sama 4 DSP per perkalian), jadi keputusannya cukup dari sisi akurasi dan range. Usulan: pakai **Q16.16** sebagai bitstream utama karena range integernya lebih lega untuk skenario di luar design envelope. Kalau tim memilih Q8.24, cukup ganti argumen `q16_16` → `q8_24` di semua perintah.

Setiap kali orang pertama mengubah ROM, bobot, atau N_ITER (misalnya setelah `generate_mpc_constants.py` dijalankan ulang):
```bash
cd "/d/ITB/Semester 5/panana/rtl/UAV/mpc_accel_hw"
python scripts/gen_hw_files.py      # buat ulang mem/<fmt>/ dan deploy/sw/mpc_formats.h
```
Setelah itu ulangi tahap 4.

---

## Tahap 4: Simulasi RTL dengan test vector

```bash
export PATH=/c/msys64/ucrt64/bin:$PATH
cd "/d/ITB/Semester 5/panana/rtl/UAV/mpc_accel_hw"

sim/run_sim.sh q16_16 +NVEC=20 +STRIDE=200   # cek cepat (~20 detik), sampel dari hover/step/trajectory
sim/run_regression.sh 6                       # regresi penuh 4000 vector, Q16.16 + Q8.24, paralel
                                              # ringkasan: sim/logs/summary_<fmt>.txt
sim/run_sim.sh q16_16 +NVEC=1 +VCD            # waveform -> tb_mpc_accel.vcd, buka di GTKWave
```

Kriteria lolos: `q16_16: 4000/4000` dan `q8_24: 4000/4000 vectors bit-exact`.

Yang diuji testbench, semuanya lewat AXI4-Lite seperti PS nantinya:
- register ID dan CONFIG, readback buffer input, IRQ;
- g[0..19] **dan** z[0..19] dibandingkan bit-exact terhadap `test_vectors_<fmt>.csv` milik orang pertama;
- flag saturasi harus 0, sesuai klaim "tidak ada overflow" dari orang pertama.

Testbench membandingkan langsung dengan CSV yang sama yang dipakai `verify_test_vectors.py`, jadi hasilnya setara dengan cara verifikasi yang diminta orang pertama. Kalau ingin menjalankan script mereka juga: `pip install numpy pandas` lalu `python ../quadcopter_mpc/scripts/verify_test_vectors.py q16_16 --all`.

**Kalau ada MISMATCH:** jalankan `+NVEC=1 +VCD` pada vector yang gagal, lalu bandingkan g terlebih dahulu (Prediction). Kalau g sudah benar, masalahnya ada di loop ADMM. Model Python orang pertama berguna untuk mencetak nilai antara (rhs, u, u_hat) per iterasi.

Bukti untuk laporan: simpan `sim/logs/summary_*.txt` dan log per potongan (`sim/logs/<fmt>_chunk*.log`), plus screenshot waveform satu solve (sinyal `state`, `op_q`, `busy`, `done`).

---

## Tahap 5: Block design + bitstream (Vivado)

Di PC Vivado (folder tanpa spasi):

1. **Cek block design dulu (cepat):**
   ```
   vivado -mode batch -source vivado/build.tcl -tclargs q16_16 bd
   ```
   Setelah itu buka `vivado/build_q16_16/mpc_kv260_q16_16.xpr` di GUI → *Open Block Design*. Isinya harus:
   `zynq_ultra_ps_e_0` → `axi_smc` (SmartConnect) → `mpc_accel_0`, plus satu `proc_sys_reset` (nama `rst_ps8_0_...`), dan `irq` tersambung ke `pl_ps_irq0`.
   Di *Address Editor*: `mpc_accel_0` berada di `0xA000_0000`, range 4K.
2. **Build penuh** (sekitar 20–40 menit):
   ```
   vivado -mode batch -source vivado/build.tcl -tclargs q16_16
   ```
3. Cek hasil di `vivado/build_q16_16/out/`:
   - `INFO: WNS = ...` di log harus **≥ 0**. Kalau negatif, lihat `timing_summary.rpt`.
   - `utilization_impl.rpt`: catat angka LUT, FF, DSP, BRAM untuk tabel BAB VII.
   - `power.rpt`: catat total on-chip power.
   - File yang dibawa ke board: `mpc_accel.bit.bin`. Simpan juga `mpc_accel.xsa` untuk arsip.

**Troubleshooting:**
| Masalah | Solusi |
|---|---|
| `board part not found` / gagal download board file | Di GUI: *Tools → Vivado Store → Boards* → install *Kria KV260 Vision AI Starter Kit*, lalu jalankan ulang |
| `$readmemh` tidak menemukan file `.mem` | Pastikan `mem/q16_16/*.mem` ada (jalankan `gen_hw_files.py`), lalu rebuild |
| Error `include mpc_ops.vh` | Pastikan `hdl/mpc_ops.vh` ikut tersalin |
| WNS negatif | Turunkan `PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ` ke 75 di `build.tcl` dan `assigned-clock-rates` di overlay, lalu rebuild |
| `bootgen` gagal | Jalankan manual di folder `out/`: `bootgen -image mpc_accel.bif -arch zynqmp -process_bitstream bin` |

---

## Tahap 6: Siapkan SD card Ubuntu (boleh paralel kapan saja)

1. Di PC, unduh image **Ubuntu 22.04 LTS untuk AMD Kria KV260** dari `ubuntu.com/download/amd` (file `.img.xz`).
2. Tulis image ke microSD dengan **balenaEtcher**. Image ini ditulis ke SD card, **bukan diinstal ke PC**.
3. Colok microSD ke KV260, sambungkan Ethernet dan kabel micro-USB ke PC, lalu nyalakan adaptor 12 V.
4. Buka serial terminal (PuTTY/Tera Term) di COM port yang muncul (biasanya yang nomornya lebih besar), **115200 baud**.
5. Login `ubuntu` / `ubuntu`, kemudian ganti password.
6. Kalau board tidak boot ke Ubuntu 22.04, update boot firmware K26 dengan *Boot Image Recovery Tool* (lihat wiki Kria, "Boot Firmware Updates"), lalu ulangi.
7. Setup Kria:
   ```bash
   sudo snap install xlnx-config --classic --channel=2.x
   sudo xlnx-config.sysinit
   sudo reboot
   ```
8. Setelah reboot, cek `sudo xmutil listapps` (harus menampilkan daftar app, minimal `k26-starter-kits`) dan `ip a` (catat IP board).

---

## Tahap 7: Load bitstream ke PL + jalankan program di PS

1. Dari PC, kirim file ke board (ganti `<ip>`):
   ```bash
   cd C:/work/mpc_accel_hw
   ssh ubuntu@<ip> mkdir -p mpc
   scp -r deploy vivado/build_q16_16/out/mpc_accel.bit.bin mem/q16_16/tv.hex ubuntu@<ip>:~/mpc/
   ```
2. Di board:
   ```bash
   cd ~/mpc/deploy
   cp ../mpc_accel.bit.bin .
   bash install_on_board.sh
   ```
   Script ini meng-compile overlay (`dtc`), menyalin `.bit.bin`, `.dtbo`, dan `shell.json` ke `/lib/firmware/xilinx/mpc_accel/`, menjalankan `xmutil unloadapp` lalu `xmutil loadapp mpc_accel`, dan terakhir build program PS.
3. Cek bahwa PL sudah termuat: `sudo xmutil listapps` harus menandai `mpc_accel` sebagai *active*.
4. Jalankan uji di board:
   ```bash
   sudo sw/mpc_hw_test ../tv.hex
   ```
   Target: `4000/4000 vectors bit-exact`. Ini membuktikan hasil hardware asli sama persis dengan golden model.

**Troubleshooting:**
| Masalah | Solusi |
|---|---|
| `ID register = 0x00000000 / 0xFFFFFFFF` | Bitstream belum termuat. Ulangi `xmutil unloadapp; xmutil loadapp mpc_accel` dan cek `dmesg | tail` |
| Board hang saat akses register | Clock PL tidak hidup atau alamat salah. Cek node `clocking0` di overlay dan Address Editor (0xA000_0000) |
| `loadapp` gagal | Nama `firmware-name` di `mpc_accel.dtsi` harus sama dengan nama file `.bit.bin` |
| `open /dev/mem failed` | Jalankan dengan `sudo` |

---

## Tahap 8: Profiling di board dan bahan BAB VII

Ini sekaligus menjawab catatan orang pertama poin 1: angka profiling mereka diukur di laptop Windows, jadi WCET sebenarnya baru valid di KV260.

1. `mpc_hw_test` sudah mencetak avg, min, P99, max, jitter (SD dan peak-peak), serta rasio terhadap Ts untuk dua hal:
   - **PL solve**: dari register CYCLES (murni accelerator);
   - **PS end-to-end**: tulis 92 register + start + polling + baca z, diukur dengan `clock_gettime`.
   Data per vector tersimpan di `hw_profiling.csv` (4000 sampel, memenuhi syarat ≥ 1000 iterasi).
2. Baseline software di CPU yang sama (Cortex-A53): minta orang pertama menyiapkan build Linux untuk `quadcopter_mpc/src/main_profiling.cpp`, atau compile langsung di board:
   ```bash
   g++ -O2 -std=c++17 -I include src/main_profiling.cpp -o main_profiling && ./main_profiling
   ```
3. Tabel BAB VII yang perlu diisi:
   - Latensi: CPU A53 (software) vs PL accelerator vs end-to-end, berikut speedup;
   - Rasio terhadap Ts = 5 ms (rata-rata dan worst-case);
   - Resource utilization (dari `utilization_impl.rpt`) dan power (`power.rpt`);
   - Hasil verifikasi: simulasi 4000/4000 bit-exact dan board 4000/4000 bit-exact.
4. Gunakan istilah yang sesuai KV260: **Zynq UltraScale+ MPSoC (XCK26), Cortex-A53, M_AXI_HPM0_FPD, DSP48E2, AXI SmartConnect**.

---

## Checklist ringkas

- [ ] Tahap 1: `check_license.tcl` → PASS (screenshot)
- [ ] Tahap 4: `run_regression.sh` → Q16.16 dan Q8.24 4000/4000 PASS (simpan log)
- [ ] Sepakati format final dengan orang pertama (usulan: Q16.16)
- [ ] Tahap 5: bitstream jadi, WNS ≥ 0, catat utilization dan power
- [ ] Tahap 6: SD card Ubuntu, board boot, `xlnx-config.sysinit`, `xmutil listapps` OK
- [ ] Tahap 7: `xmutil loadapp mpc_accel` OK, `mpc_hw_test` → 4000/4000 bit-exact
- [ ] Tahap 8: `hw_profiling.csv` + baseline A53 → tabel BAB VII
