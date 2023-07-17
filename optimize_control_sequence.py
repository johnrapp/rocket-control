# TODO
# - 
# - Lægga upp github
# - Gøra scenarion + giffar

# ----------
# - Vind linjær
# - Vind arbitraty
# - Ingen gravitation efter landningspunkten
# - Delta t gånger

import numpy as np
import pandas as pd

from pyomo.environ import *
from pyomo.opt import SolverFactory

import matplotlib.pyplot as plt

import random

from constants import (
    GRAVITY_FORCE,
    THRUST_FORCE,
    SIDE_THRUST_FORCE,
    AIR_RESISTANCE_FACTOR,
    DAMPING_FACTOR,
    BIG_M,
    N_TRIG_SEGMENTS,
    HEIGHT,
    WIDTH,
    ROCKET_HEIGHT,
    OBJECTIVE_THRESHOLD
)

x_breakpoints = list(np.linspace(-np.pi, np.pi, N_TRIG_SEGMENTS + 1))

sin_breakpoints = list(np.sin(x_breakpoints))
cos_breakpoints = list(np.cos(x_breakpoints))

abs_sin_breakpoints = list(np.abs(np.sin(x_breakpoints)))
abs_cos_breakpoints = list(np.abs(np.cos(x_breakpoints)))


def optimize_control_sequence(scenario):
    model = ConcreteModel()

    add_params(model, scenario)
    add_decision_variables(model)
    add_contraints(model)

    model.obj = Objective(rule=objective_function, sense=minimize)

    # opt = SolverFactory("bonmin", executable="/Users/simohal/Downloads/Bonmin-0.99.2-mac-osx-ix86-gcc4.0.1/bin/bonmin")
    opt = SolverFactory("gurobi")
    # opt.options["max_cpu_time"] = 30
    # opt.options["tol"] = 1e-12  # Set convergence tolerance to a small value
    opt.options["NonConvex"] = 2
    opt.options["TimeLimit"] = scenario["solve_limit"]
    opt.options['SolutionLimit'] = 1

    # opt.options['MIPGap'] = 0.1
    # opt.options['FeasibilityTol'] = 1e-2  # Allow a small violation of constraints
    try:
        results = opt.solve(model)#, tee=True)
    
        output_model(model)
    
        print(results.solver)
        if "infeasible" in str(results.Solver[0]["Termination condition"]) or model.obj() > OBJECTIVE_THRESHOLD:
            return None
        
        return get_model_vars(model)
    except Exception as e:
        print("Caught exception: ", e)
        return None

    # output_keys = [
    #     "thrust",
    #     "side_thrust",
    #     "x",
    #     "y",
    #     "rotation",
    # ]


    # return {output_key: list(model.get_component(output_key).extract_values().values()) for output_key in output_keys}
    # return control_sequence, ys, velocities


def add_params(
    m, scenario,
):
    m.x0 = scenario["x0"]
    m.y0 = scenario["y0"]
    m.x_target = scenario["x_target"]
    m.y_target = scenario["y_target"]
    m.time_steps = scenario["time_steps"]

    m.wind_direction_x = scenario["wind_direction_x"]
    m.wind_direction_y = scenario["wind_direction_y"]

    m.velocity_x0 = scenario["velocity_x0"]
    m.velocity_y0 = scenario["velocity_y0"]
    m.rotation0 = scenario["rotation0"]
    m.rotational_velocity0 = scenario["rotational_velocity0"]

    m.gravity_force = GRAVITY_FORCE
    m.thrust_force = THRUST_FORCE
    m.side_thrust_force = SIDE_THRUST_FORCE
    m.air_resistance_factor = AIR_RESISTANCE_FACTOR
    m.damping_factor = DAMPING_FACTOR


def add_decision_variables(m):
    m.t = RangeSet(1, m.time_steps)
    m.t1 = RangeSet(2, m.time_steps)

    m.thrust = Var(m.t, within=NonNegativeReals, bounds=(0, 1), initialize=0)
    m.side_thrust = Var(m.t, within=Integers, bounds=(-1, 1), initialize=0)

    m.velocity_x = Var(m.t, within=Reals, bounds=(-50, 50), initialize=0)
    m.velocity_y = Var(m.t, within=Reals, bounds=(-50, 50), initialize=0)

    m.rotation = Var(m.t, within=Reals, bounds=(-np.pi, np.pi), initialize=0)
    m.rotational_velocity = Var(m.t, within=Reals, bounds=(-np.pi, np.pi), initialize=0)

    m.x = Var(m.t, within=Reals, bounds=(0, WIDTH), initialize=0)
    m.y = Var(m.t, within=Reals, bounds=(ROCKET_HEIGHT / 2, HEIGHT), initialize=0)

    m.sin_rotation = Var(m.t, within=Reals, bounds=(-1, 1), initialize=0)
    m.cos_rotation = Var(m.t, within=Reals, bounds=(-1, 1), initialize=0)

    m.abs_sin_rotation = Var(m.t, within=Reals, bounds=(0, 1), initialize=0)
    m.abs_cos_rotation = Var(m.t, within=Reals, bounds=(0, 1), initialize=0)

    m.x[1].fix(m.x0)
    m.y[1].fix(m.y0)

    m.velocity_x[1].fix(m.velocity_x0)
    m.velocity_y[1].fix(m.velocity_y0)
    m.rotational_velocity[1].fix(m.rotational_velocity0)
    m.rotation[1].fix(m.rotation0)


def bind_thrust_to_velocity_x(m, t):
    thrust_component = m.sin_rotation[t - 1] * m.thrust[t - 1] * m.thrust_force

    relative_velocity = m.velocity_x[t] - m.wind_direction_x
    air_component = -relative_velocity * m.abs_cos_rotation[t] * m.air_resistance_factor

    return m.velocity_x[t] == m.velocity_x[t - 1] + thrust_component + air_component


def bind_thrust_to_velocity_y(m, t):
    thrust_component = m.cos_rotation[t - 1] * m.thrust[t - 1] * m.thrust_force

    relative_velocity = m.velocity_y[t] - m.wind_direction_y
    air_component = -relative_velocity * m.abs_sin_rotation[t] * m.air_resistance_factor
    gravity_component = m.gravity_force

    return (
        m.velocity_y[t]
        == m.velocity_y[t - 1] + thrust_component + gravity_component + air_component
    )

    # return (
    #     m.velocity_y[t]
    #     == m.velocity_y[t - 1] * m.air_resistance_factor
    #     + m.cos_rotation[t - 1] * m.thrust[t - 1] * m.thrust_force
    #     + m.gravity_force
    # )  # * (1 - m.landed[t])


def bind_velocity_to_pos_x(m, t):
    return m.x[t] == m.x[t - 1] + m.velocity_x[t]


def bind_velocity_to_pos_y(m, t):
    return m.y[t] == m.y[t - 1] + m.velocity_y[t]


def bind_rotation_to_rotational_velocity(m, t):
    return (
        m.rotation[t] == m.rotation[t - 1] * m.damping_factor + m.rotational_velocity[t]
    )


def bind_rotational_velocity(m, t):
    return (
        m.rotational_velocity[t]
        == m.rotational_velocity[t - 1] + m.side_thrust[t - 1] * m.side_thrust_force
    )


def add_contraints(m):
    m.bind_thrust_to_velocity_x = Constraint(m.t1, rule=bind_thrust_to_velocity_x)
    m.bind_thrust_to_velocity_y = Constraint(m.t1, rule=bind_thrust_to_velocity_y)
    m.bind_velocity_to_pos_x = Constraint(m.t1, rule=bind_velocity_to_pos_x)
    m.bind_velocity_to_pos_y = Constraint(m.t1, rule=bind_velocity_to_pos_y)

    m.bind_rotation_to_rotational_velocity = Constraint(
        m.t1, rule=bind_rotation_to_rotational_velocity
    )
    m.bind_rotational_velocity = Constraint(m.t1, rule=bind_rotational_velocity)

    # m.landing_once = Constraint(m.t1, rule=lambda m, t: m.landed[t] >= m.landed[t - 1])

    # # Add constraint for y > ROCKET_HEIGHT/2 when landed = 0
    # m.above_ground = Constraint(m.t, rule=lambda m, t: m.y[t] >= ROCKET_HEIGHT / 2 + BIG_M * (1 - m.landed[t]))

    # # Add constraint for y <= ROCKET_HEIGHT/2 when landed = 1
    # m.on_ground = Constraint(m.t, rule=lambda m, t: m.y[t] <= ROCKET_HEIGHT / 2 + BIG_M * m.landed[t])
    
    # m.landed_if_y_0 = Constraint(m.t, rule=lambda m, t: m.landed[t] * m.y[t] >= )

    #m.ensure_x_target = Constraint(
    #    rule=lambda m, t: (m.x[m.time_steps] - m.x_target) ** 2 <= 1 ** 2
    #)
    #m.ensure_y_target = Constraint(
    #    rule=lambda m, t: (m.y[m.time_steps] - m.y_target) <= 1
    #)
    
    # m.ensure_rotation_target = Constraint(rule=lambda m, t: (m.rotation[m.time_steps] - 0)**2 <= (np.pi / 12)**2)
    # m.ensure_velocity_x_target = Constraint(rule=lambda m, t: (m.velocity_x[m.time_steps] - 0)**2 <= 2**2)
    # m.ensure_velocity_y_target = Constraint(rule=lambda m, t: (m.velocity_y[m.time_steps] - 0)**2 <= 2**2)

    for t in m.t:
        m.add_component(
            "sin_rotation_approx_" + str(t),
            Piecewise(
                m.sin_rotation[t],
                m.rotation[t],
                pw_pts=x_breakpoints,
                pw_constr_type="EQ",
                pw_repn="INC",
                f_rule=sin_breakpoints,
            ),
        )
        m.add_component(
            "cos_rotation_approx_" + str(t),
            Piecewise(
                m.cos_rotation[t],
                m.rotation[t],
                pw_pts=x_breakpoints,
                pw_constr_type="EQ",
                pw_repn="INC",
                f_rule=cos_breakpoints,
            ),
        )
        m.add_component(
            "abs_sin_rotation_approx_" + str(t),
            Piecewise(
                m.abs_sin_rotation[t],
                m.rotation[t],
                pw_pts=x_breakpoints,
                pw_constr_type="EQ",
                pw_repn="INC",
                f_rule=abs_sin_breakpoints,
            ),
        )
        m.add_component(
            "abs_cos_rotation_approx_" + str(t),
            Piecewise(
                m.abs_cos_rotation[t],
                m.rotation[t],
                pw_pts=x_breakpoints,
                pw_constr_type="EQ",
                pw_repn="INC",
                f_rule=abs_cos_breakpoints,
            ),
        )
    # m.after_landing_thrust = Constraint(m.t1, rule=lambda m, t: m.thrust[t] <= BIG_M*(1 - m.landed[t]))

    # m.pos_zero_after_landing = Constraint(
    #     m.t1, rule=lambda m, t: m.y[t] <= BIG_M * (1 - m.landed[t])
    # )
    # m.consistent_landing_sequence = Constraint(
    #     m.t1, rule=lambda m, t: m.landed[t] >= m.landed[t - 1]
    # )

    # m.landing_time_consistency = Constraint(m.t, rule=lambda m, t: m.landing_time >= t*m.landed[t])
    # m.ensure_landing_time = Constraint(m.t, rule=lambda m, t: m.landing_time <= t + BIG_M * (1 - m.landed[t]))
    return


def objective_function(m):
    return (
        # sum(m.y[t] for t in m.t)
         (m.x[m.time_steps] - m.x_target) ** 2
        + (m.y[m.time_steps] - m.y_target) ** 2
        + ((m.rotation[m.time_steps] - 0) ** 2) * 10
        + ((m.rotational_velocity[m.time_steps] - 0) ** 2) * 10
        + (m.velocity_x[m.time_steps]) ** 2 * 10
        + (m.velocity_y[m.time_steps]) ** 2 * 10
        + sum(m.thrust[t] for t in m.t) / m.time_steps / 10000000
        + sum(m.side_thrust[t] ** 2 for t in m.t) / m.time_steps / 10000000
        # + m.landing_time
        # + BIG_M * sum((1 - m.landed[t]) for t in m.t)
        # + -(sum((m.velocity_x[t] + m.velocity_y[t] + m.rotational_velocity[t]) * m.landed[t] for t in m.t)) * BIG_M
        # + -(sum((m.x[t] - m.x_target) * m.landed[t] for t in m.t)) * BIG_M
    )
    # return sum(m.pos[t] for t in m.t) + sum(m.thrust[t] for t in m.t) * 50 + abs(m.velocity[m.time_steps]) / 100
    # return m.pos[m.time_steps] + abs(m.velocity[m.time_steps]) / 1000 + sum(m.thrust) * 50 + sum(m.pos) * 100000000


def get_model_vars(m):
    data = {}
    for var in m.component_objects(Var, active=True):
        if "INC" in str(var):  # skip auxiliary variables for Piecewise
            continue
        varobject = getattr(m, str(var))
        for index in varobject:
            data.setdefault(str(var), {})[index] = varobject[index].value

    df = pd.DataFrame(data)
    return df


def output_model(m):
    print(get_model_vars(m))

    # plt.plot(m.thrust.extract_values().values(), label="thrust")
    # plt.plot(m.velocity_y.extract_values().values(), label="vel")
    # plt.plot(np.array(list(m.x.extract_values().values())) / 250, label="x")
    # plt.plot(np.array(list(m.y.extract_values().values())) / 250, label="y")

    # plt.legend()
    # # plt.show()
    # plt.savefig("plot.png")

    # m.pos = Var(m.t, within=NonNegativeReals, bounds=(0, 500), initialize=0)

    # m.velocity = Var(m.t, within=Reals, bounds=(-100, 100), initialize=0)
    # for i in m.t1:
    #     m.velocity[i] = m.velocity[i-1] + m.thrust[i]*m.thrust_force + m.gravity_force

    # m.pos = Var(m.t, within=NonNegativeReals, bounds=(0, 500), initialize=0)
    # for i in m.t1:
    #     m.pos[i] = m.pos[i-1] + m.velocity[i]
