## Simple UAV simulator for the Skywalker X8 ##

This repository is made to showcase the aerodynamic model from the paper

K. Gryte, R. Hann, M. Alam, J. Rohác, T. A. Johansen, T. I. Fossen, [*Aerodynamic modeling of the Skywalker X8 Fixed-Wing Unmanned Aerial Vehicle*](https://folk.ntnu.no/torarnj/icuasX8.pdf), International Conference on Unmanned Aircraft Systems, Dallas, 2018

The goal is that this simulator will make it easier for the community to use the model, found in x8_param.mat. Please cite the above paper, should you chose to use the model.

The simulator contains a very basic autopilot, for demonstration purposes. I have not spent much time on tuning this, so performance is not optimal in any sense (PRs are welcome).

### Changes/notes ###
- Please note that only the aerodynamic model comes from the paper:
  - The thrust model is adapted from the Aerosonde model found in Beard & McClain. Small Unmanned Aircraft
  - The inertia is based on very rough initial tests, that we plan to redo in the future
- C_n_r has been multiplied with 6, compared to the value found from XFLR, to make the flight behaviour more realistic. It is believed that XFLR underpredicts the effect of winglets on the yaw stability.
- C_l_0 = C_n_0 = C_Y_0 = 0 due to symmetries. The values found from the wind tunnel tests are believed to come from misalignments in the mounting in the wind tunnel.
- C_m_alpha = -0.4629, C_m_0 = 0.0227, C_m_delta_e = -0.2292 to have the C_m(alpha) fit better with experience from flight testing. See the discussion in the paper.
