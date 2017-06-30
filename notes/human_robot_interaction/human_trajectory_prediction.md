# Human Trajectory Prediction in Crowded Spaces

[Social LSTM: Human Trajectory Prediction in Crowded Spaces](https://web.stanford.edu/~alahi/downloads/CVPR16_N_LSTM.pdf)

Pedestrians follow different trajectories to avoid obstacles and accommodate fellow pedestrians. Autonomous vehicles navigating such a scene should be able to anticipate the future positions of pedestrians and accordingly adjust its path to avoid collisions. The problem of trajectory prediction can be viewed as a sequence generation task, where we are interested in predicting the future trajectory of people based on their past positions. Propose a LSTM model which can learn general human movement and predict their future trajectories.

When people walk in a crowded public space such as a sidewalk, an airport terminal, or a shopping mall, they obey a large number of (unwritten) common sense rules and comply with social conventions. For instance, as they consider where to move next, they respect personal space and yield right-of-way.

### Model

Humans moving in crowded scenes adapt their motion based on the behavior of other people in their vicinity. For instance, a person could completely alter their path or stop momentarily to accommodate a group of people moving towards them. Such deviation in trajectory cannot be predicted by observing the person in isolation.

The model built can account for the behavior of other people within a large neighborhood, while predicting a person's path. This model is called the "Social" LSTM model.

#### Social LSTM

Every person has a different motion pattern: they move with different velocities, acceleration and have different gaits. Need a model which can understand and learn such person-specific motion properties from a limited set of initial observations corresponding to the person.

Long Short-Term Memory (LSTM) networks have been shown to successfully learn and generalize the properties of isolated sequences. Model uses one LSTM for each person in a scene. The LSTM learns the state of the person and predicts their future positions. The LSTM weights are shared across all the sequences.

##### Social Pooling

Normal LSTM models do consider the behavior of other sequences. Thus, to have each person's LSTM model interact with other people's models, neighboring LSTM's are connected through a pooling strategy.

To restrict the information from all neighboring states, create a compact representation by introducing "Social" pooling layers. At every time step, the LSTM cell receives pooled hidden-state information from the LSTM cells of neighbors.

Try to preserve the spatial information of pooled information through grid based pooling.
