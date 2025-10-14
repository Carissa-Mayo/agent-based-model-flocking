# Flocking Model with Random Walk and Predator Implementation

Agent-based flocking with cohesion, separation, alignment, optional field-of-vision (FOV), and an optional predator. Periodic boundaries, live plotting, and a clean parameter interface.

Class proejct for CSCI 4314 Dynamic Models in Biology by Dr. Orit Peleg. Code was adpated from in-class assignments.

## Model Contents

* Cohesion (attraction): mild pull toward neighbors
* Separation (repulsion): inverse-square push to avoid collisions (with epsilon guard)
* Alignment:
+ Local (FOV): align only to neighbors within a radius and angular cone
+ Or Global: align to average velocity of the flock (toggle)
* Predator (optional):
+ Boids repel from predator (inverse-square)
+ Predator seeks prey using its own FOV (narrower angle, larger radius)
* Random walk: zero-mean Gaussian velocity noise each step → stochastic drift on a torus
* Torus wrapping: positions wrap in both axes
* Speed caps: boids and predator can have different vlimit

**Random walk note**
Velocity noise is added as v_noise = c_noise*randn(2,N) each step. With all social gains set to zero, this reduces to a pure random walk (on a torus) with speed limiting. Use rng(seed) for reproducible paths.

## Results
* FOV removes unrealistic, instantaneous re-heading; turns emerge gradually and locally.
* With low/no global heading, FOV still produces small, coherent groups; the basic model does not.
* Predator + FOV yields plausible chase/avoid dynamics (predator narrows to a target; flock forms avoidance structures).

*See the report for methods, figures and discussion.*
