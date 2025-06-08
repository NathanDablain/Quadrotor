# Use -p option when calling to keep plots on screen, TODO: add axis labels and legends

set term png
set output "Position.png"
set xlabel "Time (s)"
set ylabel "Position (m)"
set yrange [-10:10]
# set multiplot layout 3,1 rows
# plot 'Sim_log.txt' using 1:2 title "North-sim", 'MCU_log.txt' using 1:3 title "North-mcu"
# plot 'Sim_log.txt' using 1:3 title "East-sim", 'MCU_log.txt' using 1:4 title "East-mcu"
plot 'Sim_log.txt' using 1:4 with lines title "Down-sim" linewidth 5, 'MCU_log.txt' using 1:5 with lines title "Down-mcu" linewidth 5, 'MCU_log.txt' using 1:22 with lines title "Down-desired" linewidth 5
# unset multiplot
unset term

set term png
set output "Euler_Angles.png
set xlabel "Time (s)"
set ylabel "Euler Angles (rad)"
set yrange [-1:1]
set multiplot layout 3,1 rows
plot 'Sim_log.txt' using 1:6 with lines title "Roll-sim" linewidth 4, 'MCU_log.txt' using 1:7 with lines title "Roll-mcu" linewidth 4
plot 'Sim_log.txt' using 1:7 with lines title "Pitch-sim" linewidth 4, 'MCU_log.txt' using 1:8 with lines title "Pitch-mcu" linewidth 4
plot 'Sim_log.txt' using 1:8 with lines title "Yaw-sim" linewidth 4, 'MCU_log.txt' using 1:9 with lines title "Yaw-mcu" linewidth 4
unset multiplot
unset term

set term png
set output "Motor_Thrusts.png"
set xlabel "Time (s)"
set ylabel "Motor Thrusts (N)"
set yrange [0:20]
set multiplot layout 2,2 rows
plot 'Sim_log.txt' using 1:15 title "Back"
plot 'Sim_log.txt' using 1:16 title "Left"
plot 'Sim_log.txt' using 1:17 title "Right"
plot 'Sim_log.txt' using 1:18 title "Front"
unset multiplot
unset term