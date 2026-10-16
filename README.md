# Actuator control robustness: scoring a controller outside the conditions it was tuned for

Tuning a PID on the nominal plant is a solved exercise. The question that
decides whether a joint works on a real machine is what happens when the
plant is not the one you tuned against: a heavier payload, friction that
rose as the gearbox wore, a noisier encoder, a control loop that picked up
latency somewhere in the stack.

So this repo does not report a step response. It tunes two controllers,
then scores both across a grid of perturbed conditions and reports **how
they fail**, not just whether they passed.

Simulated, deliberately, so ground truth is known and conditions are
repeatable. The plant model is in `src/plant.py` and is the first thing to
argue with.

## The plant

A geared DC actuator with the parts that usually get left out:

- voltage saturation, so torque is limited and the integrator can wind up
- viscous **and** Coulomb friction, including a stiction dead zone at rest
- a velocity limit
- continuous dynamics integrated with RK4 at 5 kHz while the controller
  runs at 200 Hz, because a controller does not see the world continuously
- optional measurement noise and whole step measurement delay

## Nominal results

1 rad step. Two tunes of the same controller.

| controller | rise time (s) | overshoot (%) | settling (s) | steady state error (rad) |
|---|---|---|---|---|
| conservative | 0.115 | 5.6 | 1.155 | 0.00528 |
| aggressive | 0.035 | 8.6 | 0.535 | 0.00021 |

![nominal](plots/01_nominal.png)

## Robustness results

25 perturbed cases per controller across five axes: inertia, friction,
sensor noise, sensing latency, external load.

| controller | pass rate | dominant failure mode |
|---|---|---|
| conservative | 8 % | slow to settle (19 of 25) |
| aggressive | 72 % | overshoot (5 of 25) |

![degradation](plots/02_degradation.png)

## Three findings

**1. The cautious tune is the fragile one.** Low gains look safer in the
step response and score far worse across the grid, failing 23 of 25 cases.
The reason is not instability, it is that Coulomb friction leaves a
persistent position error that weak integral action never pulls out, so the
response never enters the settling band. The failure is invisible if you
only look at overshoot, which is what "conservative tuning" is usually
protecting against. Pick your criteria before you pick your gains.

**2. Latency dominates everything else.** Scaling inertia by 8, a payload
eight times heavier than designed for, takes overshoot from 5.6 % to 38 %.
Adding 8 control steps of measurement delay, which is 40 ms, takes it to
**158 %**. Both controllers degrade almost identically, because this is not
a tuning problem, it is phase margin being spent on transport delay.

An engineering implication worth stating: if you are choosing between a
slower sensor with better accuracy and a faster one with more noise, this
plant wants the fast one. Noise at 0.02 rad standard deviation barely moved
the metrics. Latency wrecked them.

**3. Steady state error falls as load increases**, for the conservative
controller, from 0.0053 rad at no load to 0.0029 rad at 0.2 Nm. That looks
wrong and is not. The error at rest is a stiction dead zone: the controller
is commanding torque that cannot break static friction. A constant external
load biases the actuator out of that dead zone, so a disturbance improves
the number. Any metric that rewards this is measuring the wrong thing, which
is why the report keeps a failure taxonomy rather than one score.

## The harness

The part I would reuse on a real system is `evaluate.py`: a controller is
scored across a condition grid, each case is classified into a named
failure mode rather than pass or fail, and the result is a table you can
compare across changes.

```python
verdicts = ["pass", "overshoot", "slow_to_settle",
            "steady_state_error", "did_not_reach_target"]
```

A pass rate alone would say the aggressive tune is 72 % and the conservative
one is 8 %. The taxonomy says *why*, and the why is what tells you whether
to retune, add feedforward, or go and fix the sensor.

## Run it

```bash
pip install -r requirements.txt
python src/evaluate.py     # plots + data/robustness_report.json
pytest -q                  # tests, including physics sanity checks
```

## What I would do next

- Replace the PID with LQR on the linearised plant and compare degradation
  curves rather than nominal step responses.
- Add a friction observer and feedforward compensation, which is the right
  fix for finding 3 rather than tuning around it.
- Model backlash properly, which the parameter exists for and the dynamics
  currently ignore.
- Run the same harness against a real actuator on a bench and compare where
  the simulated grid predicted the failure correctly.

## Layout

```
src/plant.py      actuator dynamics, friction, saturation, RK4
src/control.py    PID with anti windup, the control loop, step metrics
src/evaluate.py   robustness grid, failure taxonomy, plots
tests/            physics sanity checks and controller behaviour
```
