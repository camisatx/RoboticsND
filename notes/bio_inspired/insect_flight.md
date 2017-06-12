# Notes on Robotic Insect Flight

## The First Takeoff of a Biologically Inspired At-Scale Robotic Insect
[Paper](http://www.micro.seas.harvard.edu/papers/TRO08_Wood.pdf)

Topic: Create microrobotic device that is capable of Diptera-like wing trajectories

### Diptera Flight
Generate aerodynamic forces with a three degree-of-freedom wing trajectory that consists of:
* A large wing stroke (defines the stroke plane)
* Wing rotation (pronation and supination) about a longitudinal wing axis
* Stroke plane deviation

Some hovering Dipteran insects that have a nearly horizontal stroke plane appear to not rely on stroke plane deviation for significant lift generation, thus it can be ignored in these situations when modeling lift.

### Robot Components
The Harvard Microrobotic Fly was reproduced aspects of Bipteran wing motion. The four main primary mechanical components include the:
* Air frame (exoskeleton)
* Actuator (flight muscle)
* Transmission (thorax)
* Airfoils

#### Actuator
A bimorph piezoelectric clamped-free bending cantilever was used as the actuator. Piezoelectric actuators typically have high operating stresses and frequencies, however, they are dense and brittle, and typically require mechanical amplification since they achieve relatively small movements.

#### Transmission
The transmission amplifies the actuator motion from a translational input to a rotational output.

#### Airfoils
The airfoils are morphologically similar to insect wings, however, the 'veins' consists of modulus carbon fiber reinforced composite beams and the 'membrane' is polyester. The veins are arranged so that the wing is extremely rigid over the expected range of flight forces.

### Questions
1. Is there a specific scale where a fully autonomous system would work. Instead of focusing on inch sized robots, would a foot sized robot have better characteristics when flying simply due to scale?
2. What is the relationship between wing size and wing flap frequency? Probably the larger the wing is the lower the frequency required to achieve the same life.
