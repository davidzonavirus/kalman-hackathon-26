#!/bin/sh
# Compile the Kalman filter coprocessor with Quartus Prime (Lite is enough).
cd "$(dirname "$0")" || exit 1
Q=$(command -v quartus_sh || ls /opt/intelFPGA*/*/quartus/bin/quartus_sh ~/intelFPGA*/*/quartus/bin/quartus_sh 2>/dev/null | head -1)
[ -n "$Q" ] || { echo "quartus_sh not found: install Quartus Prime Lite or add it to PATH"; exit 1; }
python3 ../model/kf_isa.py --emit ../rtl || exit 1
"$Q" --flow compile kf_cpu || exit 1
cat output_files/kf_cpu.fit.summary
head -30 output_files/kf_cpu.sta.summary
