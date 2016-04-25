# Kuka LWR 4+ Gazebo Component

This packages simulates the Kuka LWR 4+ (and KRC), and has the same interface as the [lwr_fri](https://github.com/kuka-isir/lwr_hardware/tree/indigo-devel/lwr_fri) which controls the hardware.

Gravity is compensated, the commands modes are ```joint impedance```, ```cartesian impedance``` and ```joint position```.

It requires the [rtt_gazebo_embedded](https://github.com/kuka-isir/rtt_gazebo_embedded) component that basically just launches a gazebo instance.

