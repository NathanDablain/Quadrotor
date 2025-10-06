# Use -p option when calling to keep plots on screen, TODO: add axis labels and legends

set term png
set output "Height.png"
set xlabel "Time (s)"
set ylabel "Height (m)"
set yrange [-1:10]
plot 'Sim_log.txt' using 1:4 with lines title "Height-sim" linewidth 5,  'PIC_log.txt' using 1:9 with lines title "Height-Filter-PIC" linewidth 5#, 'PIC_log.txt' using 1:5 with lines title "Height-PIC" linewidth 5
unset term

set term png
set output "Height_Dot.png"
set xlabel "Time (s)"
set ylabel "Height (m)"
set yrange [-2:2]
plot 'Sim_log.txt' using 1:7 with lines title "Velocity-sim" linewidth 5, 'PIC_log.txt' using 1:10 with lines title "Velocity-Filter-PIC" linewidth 5#, 'PIC_log.txt' using 1:8 with lines title "Velocity-PIC" linewidth 5
unset term

set term png
set output "Roll_Angle.png
set ylabel "Roll Angle (deg)"
set yrange [-25:25]
plot 'Sim_log.txt' using 1:9 with lines title "Roll-sim" linewidth 4, 'PIC_log.txt' using 1:12 with lines title "Roll-measured" linewidth 4
unset term

set term png
set output "Pitch_Angle.png
set ylabel "Pitch Angle (deg)"
set yrange [-25:25]
plot 'Sim_log.txt' using 1:10 with lines title "Pitch-sim" linewidth 4, 'PIC_log.txt' using 1:13 with lines title "Pitch-measured" linewidth 4
unset term

set term png
set output "Yaw_Angle.png
set ylabel "Yaw Angle (deg)"
unset yrange
plot 'Sim_log.txt' using 1:11 with lines title "Yaw-sim" linewidth 4, 'PIC_log.txt' using 1:14 with lines title "Yaw-measured" linewidth 4
unset term

set term png
set output "Motor_Thrusts.png"
set ylabel "Motor Thrusts (N)"
set yrange [0:20]
set multiplot layout 2,2 rows
plot 'Sim_log.txt' using 1:18 title "Back"
plot 'Sim_log.txt' using 1:19 title "Left"
plot 'Sim_log.txt' using 1:20 title "Right"
plot 'Sim_log.txt' using 1:21 title "Front"
unset multiplot
unset term

set term png
set output "Angular_Rate_x.png
set ylabel "Angular Rate x (deg/s)"
set yrange [-20:20]
plot 'Sim_log.txt' using 1:12 with lines title "wx-sim" linewidth 2, 'PIC_log.txt' using 1:15 with lines title "wx-pic" linewidth 2
unset term

set term png
set output "Angular_Rate_y.png
set ylabel "Angular Rate y (deg/s)"
set yrange [-20:20]
plot 'Sim_log.txt' using 1:13 with lines title "wy-sim" linewidth 2, 'PIC_log.txt' using 1:16 with lines title "wy-pic" linewidth 2
unset term

set term png
set output "Angular_Rate_z.png
set ylabel "Angular Rate z (deg/s)"
set yrange [-20:20]
plot 'Sim_log.txt' using 1:14 with lines title "wz-sim" linewidth 2, 'PIC_log.txt' using 1:17 with lines title "wz-pic" linewidth 2
unset term

set term png
set output "Body_Velocity_x.png
set xlabel "Time (s)"
set ylabel "Velocity x (m/s)"
set yrange [-5:5]
plot 'Sim_log.txt' using 1:15 with lines title "u-sim" linewidth 2, 'PIC_log.txt' using 1:18 with lines title "u-pic" linewidth 2
unset term

set term png
set output "Body_Velocity_y.png
set ylabel "Velocity y (m/s)"
set yrange [-5:5]
plot 'Sim_log.txt' using 1:16 with lines title "v-sim" linewidth 2, 'PIC_log.txt' using 1:19 with lines title "v-pic" linewidth 2
unset term