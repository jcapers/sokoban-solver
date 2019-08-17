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
            self.cost = self.parent.cost + self.distance_to_parent()

    def distance_to_parent(self):
        """
        Computes the distance from this state, to another state.

        Distance increments at least 1 since the player always moves on each step.
        If a box has been successfully moved, then the box has moved by a distance of 1 as well.

        For the purposes of the Sokoban Solver, distance g(n) is only relevant to the immediate parent.

        :return: Distance cost from parent to this State.
        """
        distance = 1
        # Get box cost, if a box has moved from last state then increment cost.
        for pos in self.parent.box_positions:
            if pos not in self.box_positions:
                distance += 1
        return distance

    def __lt__(self, o):
        return self.cost < o.cost

    def __eq__(self, o):
        return self.box_positions == o.box_positions and self.player_position == o.player_position
