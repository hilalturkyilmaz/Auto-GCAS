'running the gcas system, producing a 3d animation then saving as .gif'

import math
import sys
# sys.path.append('C:\\Users\\hilal\\OneDrive\\Masaüstü\\hilal_GCAS\\code')
sys.path.append('C:\\Users\\PC_4232_ADMIN\\Desktop\\hilal_GCAS\\code')

import imageio
import os
from numpy import deg2rad
import matplotlib.pyplot as plt

from configs.gcas_autopilot import GcasAutopilot
from configs.run_f16_sim import run_f16_sim
from functions.util import SafetyLimits, SafetyLimitsVerifier
from visualize import anim3d
from visualize import plot

# this function simulates the system and returns the simulation results.
def simulate():

    # initial conditions
    power = 9 # the engine power level of the simulated aircraft (0 - 10)

    # default alpha and beta
    alpha = deg2rad(2.1215)   # trim Angle of Attack (rad)
    beta = 0                  # side slip angle (rad)

    # initial attitude
    alt = 6500                # altitude (ft)
    vt = 540                  # initial velocity (ft/sec)
    phi = 0                   # roll angle from wings level (rad)
    theta = (-math.pi/2)*0.7  # pitch angle from nose level (rad)
    psi = 0.8 * math.pi       # yaw angle from North (rad)

    # building initial condition vectors
    # state = [vt, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow]
    init = [vt, alpha, beta, phi, theta, psi, 0, 0, 0, 0, 0, alt, power] ### Sensors 1st component of Auto-GCAS ###

    tmax = 15 # simulation time

    ap = GcasAutopilot(init_mode='waiting', gain_str='old', stdout=True) ### Trajectory Prediction Algorithm 2nd component of Auto-GCAS ###

    ap.waiting_time = 5
    ap.waiting_cmd[1] = 2.2 # ps command
    ap.cfg_k_prop = 1.4
    ap.cfg_k_der = 0
    ap.cfg_eps_p = deg2rad(20)
    ap.cfg_eps_phi = deg2rad(15)
    print("TERRAIN TERRAIN!") ### Displays/Annunciators 3rd component of Auto-GCAS ###
    print("PULL UP!")

    assert tmax > ap.waiting_time, "tmax must be greater than ap.waiting_time"

    step = 1/30
    
    res = run_f16_sim(init, tmax, ap, step=step, extended_states=True, model_str='morelli', integrator_str='rk45', v2_integrators= False)

    print(f"Simulation Completed in {round(res['runtime'], 2)} seconds")

    # safety limits determine whether the GCAS system kept the plane in a safe state. Auto-GCAS
    safety_limits = SafetyLimits( \
        altitude=(0, 45000), #ft 
        Nz=(-5, 18),         #G's 
        v=(300, 2500),       # ft/s 
        alpha=(-10, 45),     # deg 
        betaMaxDeg=30,       # deg
        psMaxAccelDeg=500)   # deg/s/s

    verifier = SafetyLimitsVerifier(safety_limits, ap.llc)
    verifier.verify(res)

    plot.plot_single(res, 'alt', title='Altitude (ft)')
    filename = 'AGCAS_altitude.png'
    plt.savefig(filename)
    print(f"{filename} saved.")

    plot.plot_attitude(res)
    filename = 'AGCAS_attitude.png'
    plt.savefig(filename)
    print(f"{filename} saved.")

    # plot inner loop controls + references
    plot.plot_inner_loop(res)
    filename = 'AGCAS_inner_loop.png'
    plt.savefig(filename)
    print(f"{filename} saved.")

    # plot outer loop controls + references
    plot.plot_outer_loop(res)
    filename = 'AGCAS_outer_loop.png'
    plt.savefig(filename)
    print(f"{filename} saved.")

    return res

# this function performs simulation, generates 3D plots, and creates animations based on the simulation results.
def save():
    'main function'

    if len(sys.argv) > 1 and (sys.argv[1].endswith('.gif')):
        filename = sys.argv[1]
        print(f"saving result to '{filename}'")
    else:
        filename = 'AGCAS_output.gif'

    res = simulate()
    
    anim3d.make_anim(res, filename, elev=15, azim=-150)

    print('AGCAS_output.gif saved.')

def save_frames_from_gif(gif_filename, output_folder):
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Load GIF file
    gif = imageio.get_reader(gif_filename)

    # Iterate over each frame and save it as a separate file
    for i, frame in enumerate(gif):
        frame_filename = os.path.join(output_folder, f"frame_{i:03d}.png")
        imageio.imwrite(frame_filename, frame)

    print('AGCAS_output.gif frames saved to the folder.')

if __name__ == '__main__':
    save()
    save_frames_from_gif('AGCAS_output.gif', 'AGCAS_output_frames_folder')
