# L2 Rocket Flight Sim

A 1D flight simulator for my Tripoli Level 2 certification rocket, written from scratch in MATLAB.

The rocket is 6 ft long with a 4 in diameter and flies on an AeroTech J540R motor. I built this sim to develop my simulation skills and gain experience with ODE solvers (`ode45`), then validated it against OpenRocket.


## What it models

- **Thrust:** linear interpolation of the J540R thrust curve, zero after burnout
- **Mass:** propellant burns off in proportion to impulse delivered, m(t) = m₀ − m_prop · I(t) / I_total
- **Atmosphere:** air density decreases with altitude (standard atmosphere, troposphere)
- **Drag:** D = ½ ρ C_D A v|v|, always opposing motion
- **Launch pad:** the rocket stays on the pad until thrust exceeds weight

The equation of motion, a = (T − W − D) / m, is split into a first-order system, y = [h; v], and solved with `ode45`.
