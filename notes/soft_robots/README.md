# Soft Robots

Soft robots contains little or no rigid materials, and are instead primarily composed of fluids, gels, soft polymers, and other easily deformable matter. These materials exhibit similar elastic properties of soft biological matter and allow the robot to remain operational even as it is stretched and squeezed.

The promise of soft robots is best realized in environments and applications that require interaction with soft materials and organisms, and the artificial replication of biological functionalities.

### Applications

In medical and personal co-robotics domains, soft-matter machines and robots allow for safe and biomechanically compatible interactions with humans.

For field exploration and disaster relief, soft robots can navigate challenging terrain and penetrate tightly confined spaces by adapting their shape and locomotion strategy in ways similar to natural organisms.

Miniature soft robots could function as artificial microorganisms in medical applications such as drug delivery, angioplasty, and biopsy.

### Challenges

The key challenge is the development of controllable soft bodies using materials that integrate sensors, actuators, and computation to deliver the desired behavior. The use of soft materials will require new algorithms to achieve these goals.

### Materials

#### Compliance Matching

Soft robots must be made from materials that have the right amount of rigidity to prevent damage to the object they are interacting with.

One measure of material rigidity is the modulus of elasticity, or Young's modulus - a quantity that scales with the ratio of force to percent elongation. Young's modulus is only defined for homogeneous, prismatic bars that are subject to axial loading and small deformations, which isn't a perfect match for soft robotic measurements but is still useful.

Based on Young's modulus, materials in natural organisms are 3 to 10 orders of magnitude less rigid that the materials in conventional robots.

Mismatches in elastic compliance can lead to damaging stress concentrations, redistribute internal forces in a way that leads to disuse atrophy of bone or tissue, or introduce rigid kinematic constraints that interfere with natural motor function.

#### Design Techniques

Soft-lithography fabrication approaches typically use a layer of stiffer rubber or elastomer (with paper, fabric or plastic film) embedded to achieve asymmetric strain for actuation. An alternative approach is to augment all elastomeric elements with flexible fibers, which limit the stress taken up by the elastomer during pneumatic actuation. This results in actuators with reduced extensibility and flexibility, but the ability to withstand higher actuation pressures, and apply larger force.

### Actuation

There are a few key ways to actuate a soft robot, with the main approaches being variable length tendons and pneumatic actuation.

#### Variable Length Tendons

Variable length tendons can be in the form of tension cables or shape memory alloy actuators. They can be embedded in soft segments.

#### Pneumatic Actuation

Pneumatic actuation uses channels in soft materials that are inflated, causing a desired deformation.

*Pneumatic artificial muscles* (PAMs), also called McKibben actuators, are examples of compliant linear soft actuators of elastomer tubes in fiber sleeves.

*Fluidic elastomer actuators* (FEAs) comprise synthetic elastomer films operated by the expansion of embedded channels under pressure. Once pressureized, the actuator will keep its position with little or no additional energy consumption. FEAs can be operated pneumatically or hydraulically. FEAs are highly extensible and adaptable, being low power soft actuators.

Pneumatic networks (pneu-nets) require high strains for actuation, leading to slow actuation rates and rupture failures.

#### Electrically Activated Actuators

*Electroactive plymers* (EAPs) use electricity to actuate their shapes. Specific types of EAPs include Dielectric EAPs, ferroelectric polymers, electrostrictive graft polymers, liquid crystal polymers, ionic EAPs, electrorheological fluids, ionic polymer-metal composites, and stimuli-responsive gels.

### Computation

Soft bodied robots cannot be confined to planar motions like rigid bodied robots can. Essentially, soft robots have an infinite number of degrees of freedom since they can bend, twist, stretch, compress, buckle and wrinkle. This requires a new approach for modeling and controlling these robots.

The inverse kinematics problem is especially challenging since neither the whole body, nor the pose of the end effector are considered in the solution. Real time, closed loop curvature controllers can be used to drive the bending of soft pneumatic body segments of a manipulator despite their high compliance and lack of kinematic constraints.

### Control

Low level control of pressure based robots using pressure transducers or volume control using strain sensors. Most fluid powered robots use open-loop valve sequencing to control body segment actuation. Valve sequencing means that a valve is turned on for some duration of time to pressurize the actuator and then turned off to either hold or deflate it.

### Sources

[Soft Robotics: A Perspective - Current Trends and Prospects for the Future](./papers/Soft_Robots_A_Perspective.pdf)

[A Resilient, Untethered Soft Robot](./papers/A_Resilient_Untethered_Soft_Robot.pdf)

[Design, Fabrication, and Control of Soft Robots](./papers/Design_Fabrication_and_Control_of_Soft_Robots.pdf)

[A Hybrid Combining Hard and Soft Robots](./papers/A_Hybrid_Combining_Hard_and_Soft_Robots.pdf)

[An Untethered Miniature Origami Robot that Self-folds, Walks, Swims, and Degrades](./papers/An_Untethered_Miniature_Origami_Robot.pdf)

[A Brief History of Oribotics](./papers/A_Brief_History_of_Oribotics.pdf)
