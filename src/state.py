from typing import List, Tuple


class State:
    """A node that holds a Sokoban State."""

    def __init__(self,
                 parent,
                 box_positions: List[Tuple[int, int]],
                 player_position: Tuple[int, int],
                 move: str = None):
        self.parent = parent
        self.box_positions = box_positions
        self.player_position = player_position
        # The move taken to get to this state.
        self.move_taken = move
        # g(n) cost to get here. Cost is 0 if this node is the root node, with parent == None.
        if self.parent is None:
            self.cost = 0
        else:
            self.cost = self.compute_cost()
        self.h_cost = 0

    def compute_cost(self):
        """
        Computes the distance from this state, to root node.
        This represents g(n) in f(n) = g(n) + h(n).

        Distance increments at least 1 since the player always moves on each step.

        :return: Distance cost from parent to this State.
        """
        cost_per_move = 1
        return self.parent.cost + cost_per_move

    def __lt__(self, o):
        return self.cost < o.cost

    def __gt__(self, o):
        return self.cost > o.cost

    def __eq__(self, o):
        return self.box_positions == o.box_positions and self.player_position == o.player_position

    def __hash__(self):
        if len(self.box_positions) > 1:
            temp = tuple(map(sum, self.box_positions))
        else:
            temp = self.box_positions[0]
        current_sum = temp[0] + self.player_position[0] + temp[1] + self.player_position[1]
        return current_sum
