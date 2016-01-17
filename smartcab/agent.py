import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # Initialize any additional variables here
        self.next_waypoint = None
        self.state = [0, 0] # (Total steps, total reward) till now

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required
        self.state = [0, 0] # (Total steps, total reward) till now
        self.next_waypoint = None

    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)
        heading = self.env.agent_states[self]['heading']
        location = self.env.agent_states[self]['location']

        # Select action
        action, new_heading = self.env.q_table.next_move(location, heading)
        # action = random.choice(Environment.valid_actions) # Select random action

        # Update state
        action_okay = True
        if action == 'right':
            if inputs['light'] == 'red' and inputs['left'] == 'forward':
                action_okay = False
        elif action == 'forward':
            if inputs['light'] == 'red':
                action_okay = False
        elif action == 'left':
            if inputs['light'] == 'red' or (inputs['oncoming'] == 'forward' or inputs['oncoming'] == 'right'):
                action_okay = False

        if not action_okay:
            action = None
            # action = self.next_waypoint
            # self.next_waypoint = random.choice(Environment.valid_actions[1:])

        # Execute action and get reward
        reward = self.env.act(self, action)
        self.env.q_table.update(location, new_heading, reward, learning_rate=0.5, discount_factor=0.8)

        if action:
            self.state[0] += 1
            self.state[1] += reward

        # TODO: Learn policy based on state, action, reward

        # print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}, total reward = {}".format(deadline, inputs, action, reward, self.state[1])  # [debug]


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # set agent to track

    # Now simulate it
    sim = Simulator(e, update_delay=0)  # reduce update_delay to speed up simulation
    sim.run(n_trials=100)  # press Esc or close pygame window to quit


if __name__ == '__main__':
    run()
