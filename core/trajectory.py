import numpy as np
from scipy.interpolate import make_interp_spline

def sample_angular_trajectory(form="sinusoidal", duration=20, dt=0.005, random=False, p_zero=0.0, ramp_time=1.5):

    if (random):
        form = np.random.choice(["sinusoidal", "circular", "triangular", "factor"])
    
    t = np.arange(0, duration, dt)

    # ================= LIMITS =================
    phi_min, phi_max = np.deg2rad(-10), np.deg2rad(10)  # Pitch
    psi_min, psi_max = np.deg2rad(-25), np.deg2rad(25)  # Yaw

    traj = {}

    # ================= SINUSOIDAL =================
    def gen_sinusoidal(t, amin, amax):
        omega_dot_max = np.deg2rad(250)

        A_geom_max = 0.45 * (amax - amin)
        A = np.random.uniform(0.2 * A_geom_max, A_geom_max)
        nu_max = np.sqrt(omega_dot_max / (2*np.pi*A))

        nu = np.random.uniform(1.0, min(2, nu_max))

        offset = np.random.uniform(amin + A, amax - A)
        phase = np.random.uniform(0, 2*np.pi)

        return offset + A * np.sin(2*np.pi*nu*t + phase)

    # ================= TRIANGULAR =================
    def gen_triangular(t, amin, amax):
        freq = np.random.uniform(1.0, 1.5)                     
        full_amp = (amax - amin) / 2 
        amp = np.random.uniform(0.2, 1.0) * full_amp
        offset = (amax + amin) / 2
        return offset + amp * (2*np.abs(2*((t*freq) % 1) - 1) - 1)
 
    # ================= RANDOM =================
    def gen_bezier(t, amin, amax):
        n_points = np.random.randint(20, 100)
        tp = np.linspace(0, t[-1], n_points)
        control = np.random.uniform(amin, amax, size=n_points)
        spline = make_interp_spline(tp, control, k=3)
        return spline(t)

    # ================= CIRCULAR =================
    def gen_circular(t):
        R = np.deg2rad(np.random.uniform(
            1.0,
            0.65 * min(
                np.rad2deg(phi_max - phi_min),
                np.rad2deg(psi_max - psi_min)
            )
        ))
        f = np.random.uniform(1.0, 1.5)
        
        phi0 = np.random.uniform(phi_min + R, phi_max - R)
        psi0 = np.random.uniform(psi_min + R, psi_max - R)

        ph = np.random.uniform(0, 2*np.pi)

        pitch = phi0 + R * np.cos(2*np.pi*f*t + ph)
        yaw   = psi0 + R * np.sin(2*np.pi*f*t + ph)

        return pitch, yaw
    
    # ================= LOOPS =================
    def gen_infinite_like_random_factor(t):
     
        f_max = 1.5
        
        A_pitch = np.random.uniform(0.25 * (phi_max - phi_min), 0.5 * (phi_max - phi_min))
        A_yaw   = np.random.uniform(0.25 * (phi_max - phi_min), 0.5 * (psi_max - psi_min))
 
        pitch_offset = np.random.uniform(phi_min + A_pitch, phi_max - A_pitch)
        yaw_offset   = np.random.uniform(psi_min + A_yaw,   psi_max - A_yaw)
        
        factors = [0.5, 2.0]
        factor = np.random.choice(factors)
        
        apply_to_pitch = np.random.rand() < 0.5
        
        if apply_to_pitch:
            f_pitch = np.random.uniform(0.8, f_max)
            f_yaw = f_pitch / factor if factor >= 1 else f_pitch * factor
            if f_yaw > f_max:
                f_yaw = f_max
                f_pitch = f_yaw * factor if factor >= 1 else f_yaw / factor
        else:
            f_yaw = np.random.uniform(0.8, f_max)
            f_pitch = f_yaw / factor if factor >= 1 else f_yaw * factor
            if f_pitch > f_max:
                f_pitch = f_max
                f_yaw = f_pitch * factor if factor >= 1 else f_pitch / factor
        
        # phase_pitch = np.random.uniform(0, 2*np.pi)
        # phase_yaw   = np.random.uniform(0, 2*np.pi)
        phase_pitch = 0
        phase_yaw   = 0
        
        pitch = pitch_offset + A_pitch * np.sin(2*np.pi * f_pitch * t + phase_pitch)
        yaw   = yaw_offset   + A_yaw   * np.sin(2*np.pi * f_yaw   * t + phase_yaw)
        
        return pitch, yaw
    
    # ================= GENERATORS =================
    generators = {
        "sinusoidal": gen_sinusoidal,
        "triangular": gen_triangular,
        "bezier": gen_bezier,
        "circular": gen_circular,
        "factor": gen_infinite_like_random_factor
    }

    if form not in generators:
        raise ValueError(f"Form '{form}' not recognized. Use one of: {list(generators.keys())}")

    # ================= FORM SELECT =================
    if form == "circular":
        pitch, yaw = gen_circular(t)
    elif form == "factor":
        pitch, yaw = gen_infinite_like_random_factor(t)
    else:
        gen_func = generators[form]
        pitch = gen_func(t, phi_min, phi_max)
        yaw   = gen_func(t, psi_min, psi_max)
    
    # ================= ZERO PROB =================
    if np.random.rand() < p_zero:
        if np.random.rand() < 0.5:
            pitch = np.zeros_like(t)  
        else:
            yaw = np.zeros_like(t)    

    # ================= RAMP UP TIME =================
    def add_linear_start(sig, t, ramp_time):
        dt = t[1] - t[0]
        n_ramp = int(ramp_time / dt)
        start_val = sig[0]
        ramp = np.linspace(0, start_val, n_ramp)
        sig_new = np.concatenate([ramp, sig])
        t_new   = np.concatenate([np.linspace(0, ramp_time, n_ramp, endpoint=False), t + ramp_time])
        return t_new, sig_new

    _, pitch = add_linear_start(pitch, t, ramp_time)
    t, yaw   = add_linear_start(yaw, t, ramp_time)

    traj["pitch"] = pitch
    traj["yaw"] = yaw

    return t, traj

def generate_trajectory(form: str, duration=20, dt=0.005, vel=1, 
                        center=(0,0), radius=1, scale=1):

    t = np.arange(0, duration, dt)
    w = 2 * np.pi * vel  
    y0, z0 = center

    if form.lower() == 'circle':
        y = y0 + radius * np.cos(w * t)
        z = z0 + radius * np.sin(w * t)

    elif form.lower() == 'corazon':
        theta = w * t
        y = y0 + scale * 16 * (np.sin(theta))**3 / 17
        z = z0 + scale * (13*np.cos(theta) - 5*np.cos(2*theta) -
                           2*np.cos(3*theta) - np.cos(4*theta)) / 17

    elif form.lower() == 'loop':
        theta = w * t
        a = scale
        y = y0 + a * np.sin(theta)
        z = z0 + a * np.sin(theta) * np.cos(theta)
    
    elif form.lower() == 'square':
        ciclos = int(duration * vel)
        points_per_circle = int(len(t) / max(ciclos, 1))

        y = np.full(len(t), y0)
        z = np.full(len(t), z0)

        for i in range(ciclos):
            ini = i * points_per_circle
            end = (i + 1) * points_per_circle if (i + 1) * points_per_circle <= len(t) else len(t)
            n = end - ini
            seg = n // 4

            y_temp = np.zeros(n)
            z_temp = np.zeros(n)

            lado = radius * scale

            y_temp[:seg] = np.linspace(y0 - lado, y0 + lado, seg)
            z_temp[:seg] = z0 - lado

            y_temp[seg:2*seg] = y0 + lado
            z_temp[seg:2*seg] = np.linspace(z0 - lado, z0 + lado, seg)

            y_temp[2*seg:3*seg] = np.linspace(y0 + lado, y0 - lado, seg)
            z_temp[2*seg:3*seg] = z0 + lado

            y_temp[3*seg:] = y0 - lado
            z_temp[3*seg:] = np.linspace(z0 + lado, z0 - lado, n - 3*seg)

            y[ini:end] = y_temp
            z[ini:end] = z_temp

    elif form.lower() in ['line_y', 'line_z', 'diagonal']:
        clycles = int(duration * vel)
        t_cycle = np.linspace(0, 1, int(len(t) / max(clycles, 1)))
        saw = np.abs(2 * (t_cycle % 1) - 1)
        saw_full = np.tile(saw, clycles + 1)[:len(t)]
        amp = radius * scale

        if form.lower() == 'line_y':
            y = y0 - amp + 2 * amp * saw_full
            z = np.full_like(y, z0)

        elif form.lower() == 'line_z':
            y = np.full_like(t, y0)
            z = z0 - amp + 2 * amp * saw_full

        elif form.lower() == 'diagonal':
            y = y0 - amp + 2 * amp * saw_full
            z = z0 - amp + 2 * amp * saw_full

    else:
        raise ValueError("Form not recognized.")

    return t, y, z